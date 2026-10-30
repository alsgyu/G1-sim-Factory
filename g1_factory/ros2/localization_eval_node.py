"""localization_eval: records the localizer and the zone it implies against the simulator's truth (simulation only).

Runs in launch/g1_localization_eval.launch.py: the robot walks a route on the simulator's map (mapping_tour,
global_planner, path_follower), and the localizer runs on the robot's map package with publish_tf:=false, so the
measurement never steers the robot. Raw records (scripts/eval_localization.py computes the metrics from them):

  samples.csv     sample_hz rows: localised pose (package map frame) moved into the simulator frame with the
                  package's world_from_map vs the true pose at the same stamp; the zone the robot would say it is in
                  (localised pose + annotated room bounds, region_at as in vln_navigator) vs the simulator's zone at
                  the true pose
  fix_trials.csv  every trial_every_m metres of walking, two fresh localiser searches on the current scan from a
                  rough guess: one inside the search window (0.5-1.5 m, 0-20 deg off) and one outside (2.5-5 m,
                  0-30 deg); result correct / refused / wrong, error, inlier share, time
  status.jsonl    /localization/status (first fix, lost events)
  run.json        parameters, world_from_map, start and end
Ends on /mapping/done (end of the route): writes the files and exits, which ends the launch.
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_factory import slam_map as S
from g1_factory.localization import Localizer, compose, inverse, wrap
from g1_factory.map_package import load_package, vln_inputs, world_from_map
from g1_factory.roadmap import region_at
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]


def nearest(buf, t, tol=0.06):
    """Entry of a (t, ...) deque closest to time t, or None when further than tol."""
    if not buf:
        return None
    best = min(buf, key=lambda s: abs(s[0]-t))
    return best if abs(best[0]-t) <= tol else None


class LocalizationEval(Node):
    def __init__(self):
        super().__init__("localization_eval")
        p = self.declare_parameter
        self.package = Path(p("package", "").value)
        self.out = Path(p("output_dir", str(ROOT/"outputs/metrics/localization")).value)
        self.out.mkdir(parents=True, exist_ok=True)
        truth_semantic = Path(p("truth_semantic", str(ROOT/"outputs/isaac/semantic_map.json")).value)
        truth_topic = p("truth_topic", "/odom").value
        self.sample_period = 1.0/float(p("sample_hz", 5.0).value)
        self.trial_every = float(p("trial_every_m", 3.0).value)
        self.windows = {"in_window": tuple(p("trial_in_window", [0.5, 1.5, 20.0]).value),
                        "out_of_window": tuple(p("trial_out_of_window", [2.5, 5.0, 30.0]).value)}
        seed = int(p("seed", 0).value)
        self.rng = np.random.default_rng(seed)
        W = world_from_map(self.package)
        self.W = (float(W[0, 2]), float(W[1, 2]), math.atan2(W[1, 0], W[0, 0]))
        self.map_from_world = inverse(self.W)
        grid, origin, res, semantic = load_package(self.package, "map")
        self.rooms = vln_inputs(semantic)[0]  # what vln_navigator uses for "You are in: ..."
        if truth_semantic.is_file():
            truth = json.loads(truth_semantic.read_text())
        else:  # the package's copy of the simulator's semantic map, moved back into the simulator frame
            truth = S.transform_semantic(json.loads((self.package/"semantic_truth.json").read_text()), W)
        self.truth_zones = {z["id"]: tuple(z["bounds"]) for z in truth["zones"]}
        self.trial_loc = Localizer(grid, origin, res, seed=seed)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        self.create_subscription(Odometry, truth_topic, self.on_truth, TOPICS["/odom"][1])
        if truth_topic != "/odom":
            sub("/odom", Odometry, self.on_odom)
        sub("/localization/pose", PoseWithCovarianceStamped, self.on_pose)
        sub("/localization/status", String, self.on_status)
        sub("/livox/lidar", PointCloud2, self.on_cloud)
        self.create_subscription(String, "/mapping/done", self.on_done, TOPICS["/vln/instruction"][1])
        self.truth, self.odom = deque(maxlen=3000), deque(maxlen=3000)
        self.same_odom = truth_topic == "/odom"
        self.est = deque(maxlen=200)
        self.samples, self.trials, self.status = [], [], []
        self.travelled, self.last_xy, self.next_trial = 0.0, None, self.trial_every
        self.last_sample_t = -math.inf
        self.pool, self.job = ThreadPoolExecutor(1), None
        self.finished, self.started_wall = False, time.time()
        self.params = {"package": str(self.package), "truth_topic": truth_topic, "truth_semantic": str(truth_semantic),
                       "sample_hz": 1/self.sample_period, "trial_every_m": self.trial_every,
                       "windows": self.windows, "world_from_map": self.W}
        self.create_timer(self.sample_period, self.sample)
        self.get_logger().info(f"measuring the localizer on {self.package} against the truth ({truth_topic})")

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ inputs
    def on_truth(self, msg):
        t, p = M.to_seconds(msg.header.stamp), msg.pose.pose.position
        pose = (p.x, p.y, M.quaternion_yaw(msg.pose.pose.orientation))
        self.truth.append((t, *pose))
        if self.same_odom:
            self.odom.append((t, *pose))
        if self.last_xy is not None:
            self.travelled += math.hypot(pose[0]-self.last_xy[0], pose[1]-self.last_xy[1])
        self.last_xy = pose[:2]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((M.to_seconds(msg.header.stamp), p.x, p.y, M.quaternion_yaw(msg.pose.pose.orientation)))

    def on_pose(self, msg):
        p = msg.pose.pose.position
        self.est.append((M.to_seconds(msg.header.stamp), p.x, p.y, M.quaternion_yaw(msg.pose.pose.orientation)))

    def on_status(self, msg):
        s = json.loads(msg.data)
        if not self.status or s["state"] != self.status[-1]["state"]:
            self.status.append({"t": round(self.now(), 3), **{k: s.get(k) for k in ("state", "message", "inliers",
                                                                                     "correction_m", "correction_deg")}})

    # ------------------------------------------------------------------ continuous samples
    def sample(self):
        if not self.est or self.finished:
            return
        t, ex, ey, eyaw = self.est[-1]
        if t <= self.last_sample_t:
            return
        truth = nearest(self.truth, t)
        if truth is None:
            return
        self.last_sample_t = t
        _, tx, ty, tyaw = truth
        wx, wy, wyaw = compose(self.W, (ex, ey, eyaw))
        self.samples.append([round(t, 3), round(self.travelled, 2), round(math.hypot(wx-tx, wy-ty), 4),
                             round(math.degrees(abs(float(wrap(wyaw-tyaw)))), 3), round(wx, 3), round(wy, 3),
                             round(tx, 3), round(ty, 3), region_at(self.rooms, (ex, ey)),
                             region_at(self.truth_zones, (tx, ty))])

    # ------------------------------------------------------------------ first-fix trials
    def on_cloud(self, msg):
        if self.finished or self.travelled < self.next_trial or (self.job is not None and not self.job.done()):
            return
        t = M.to_seconds(msg.header.stamp)
        truth, odom = nearest(self.truth, t), nearest(self.odom, t)
        if truth is None or odom is None:
            return
        try:
            tf = self.buffer.lookup_transform("odom", msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        self.next_trial = self.travelled+self.trial_every
        m = M.transform_matrix(tf.transform)
        xyz = M.cloud_xyz(msg)@m[:3, :3].T+m[:3, 3]
        true_map = compose(self.map_from_world, tuple(truth[1:]))
        self.job = self.pool.submit(self.run_trials, t, xyz, m[:3, 3].copy(), tuple(odom[1:]), true_map,
                                    round(self.travelled, 2))

    def run_trials(self, t, xyz, sensor, odom_pose, true_map, travelled):
        for kind, (lo, hi, max_deg) in self.windows.items():
            a = self.rng.uniform(-math.pi, math.pi)
            r = self.rng.uniform(lo, hi)
            dyaw = math.radians(self.rng.uniform(0, max_deg))*self.rng.choice([-1, 1])
            guess = (true_map[0]+r*math.cos(a), true_map[1]+r*math.sin(a), true_map[2]+dyaw)
            self.trial_loc.initialize(guess, odom_pose)
            begin = time.monotonic()
            u = self.trial_loc.update(xyz, sensor)
            latency = time.monotonic()-begin
            err = deg = None
            if u.state == "localized":
                est = self.trial_loc.map_pose(odom_pose)
                err = math.hypot(est[0]-true_map[0], est[1]-true_map[1])
                deg = math.degrees(abs(float(wrap(est[2]-true_map[2]))))
            self.trials.append([round(t, 3), travelled, kind, round(r, 3), round(math.degrees(abs(dyaw)), 2),
                                u.state == "localized", None if err is None else round(err, 4),
                                None if deg is None else round(deg, 3),
                                round(u.match.inliers, 3) if u.match else None, round(latency, 3), u.points,
                                round(true_map[0], 3), round(true_map[1], 3)])

    # ------------------------------------------------------------------ end
    def on_done(self, msg):
        self.get_logger().info("route finished; writing the measurement")
        self.finish(json.loads(msg.data) if msg.data.startswith("{") else {})
        raise SystemExit(0)

    def finish(self, route=None):
        if self.finished:
            return
        self.finished = True
        if self.job is not None:
            self.job.result()
        with (self.out/"samples.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sim_time_s", "travelled_m", "position_error_m", "yaw_error_deg", "x_m", "y_m", "true_x_m",
                        "true_y_m", "zone_robot", "zone_truth"])
            w.writerows(self.samples)
        with (self.out/"fix_trials.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sim_time_s", "travelled_m", "window", "guess_offset_m", "guess_offset_deg", "localized",
                        "error_m", "error_deg", "inliers", "latency_s", "scan_points", "true_map_x_m", "true_map_y_m"])
            w.writerows(self.trials)
        (self.out/"status.jsonl").write_text("".join(json.dumps(s)+"\n" for s in self.status))
        (self.out/"run.json").write_text(json.dumps({
            **self.params, "sim_time_end_s": round(self.now(), 2), "wall_s": round(time.time()-self.started_wall, 1),
            "travelled_m": round(self.travelled, 1), "samples": len(self.samples), "trials": len(self.trials),
            "route": route or {}}, indent=1))
        self.get_logger().info(f"{len(self.samples)} samples, {len(self.trials)} fix trials, "
                               f"{self.travelled:.0f} m walked -> {self.out}")


def main():
    rclpy.init()
    node = LocalizationEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except RuntimeError:  # rclpy can fail to take a message while the launch shuts the graph down
        if rclpy.ok():
            raise
    finally:
        node.finish()
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
