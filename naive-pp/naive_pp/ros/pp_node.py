"""naive_pp: instruction -> place on the robot's map -> topological route -> A* path on /plan.

    python3 -m naive_pp.ros.pp_node --ros-args -p package:=/ws/naive-pp/maps/campus_dev

The robot's pose comes from /odom and the localizer's TF map -> odom (LiDAR scans matched to the package's SLAM
map), never from the simulator. g1_factory's path_follower turns /plan into /cmd_vel for the G1 walking policy.

Subscribes
  /vln/instruction       std_msgs/String   text or JSON from the external agent (naive_pp.grounding)
  /odom + /tf            the robot in the map frame (g1_factory.ros2.frames.MapPose)
  /localization/status   std_msgs/String   localizer state; plans wait for "localized"
  /livox/lidar           sensor_msgs/PointCloud2   obstacles the map lacks: replan when they block the path
  /nav/status            std_msgs/String   path_follower: reached / failed for the current plan id
Publishes
  /plan                  nav_msgs/Path     map frame, 0.1 m spacing, last pose = final heading; stamp = plan id
  /vln/feedback          std_msgs/String   JSON events: accepted, waiting, planned, replanned, progress,
                                           done | failed | canceled (terminal)
  /naive_pp/goal         geometry_msgs/PoseStamped   where the robot is going (latched)
  /naive_pp/route        nav_msgs/Path     robot -> doorways -> goal of the topological route (latched)
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from naive_pp.grounding import GroundingError, load_keywords
from naive_pp.package import load_package
from naive_pp.planner import NaivePlanner, PlannerConfig, PlanningError
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import LATCHED, TOPICS

HERE = Path(__file__).resolve().parents[2]


class NaivePPNode(Node):
    def __init__(self):
        super().__init__("naive_pp")
        p = lambda name, default: self.declare_parameter(name, default).value
        package = p("package", str(HERE/"maps/campus_dev"))
        self.frame = p("map_frame", "map")
        cfg = PlannerConfig(robot_radius=float(p("robot_radius_m", 0.40)), clearance=float(p("clearance_m", 1.0)),
                            goal_clearance=float(p("goal_clearance_m", 0.8)), corridor_m=float(p("corridor_m", 1.0)),
                            goal_mode=p("goal_mode", "center"))
        self.pp = NaivePlanner(load_package(package), cfg, load_keywords(p("keywords",
                                                                           str(HERE/"configs/place_keywords.yaml"))))
        self.need_fix = bool(p("require_localization", True))
        self.goal_tolerance = float(p("goal_tolerance_m", 0.5))
        self.replan_period = float(p("replan_period_s", 2.0))
        self.max_replans = int(p("max_replans", 12))
        self.stuck_timeout = float(p("stuck_timeout_s", 45.0))
        self.max_deviation = float(p("max_deviation_m", 1.5))
        self.task_timeout = float(p("task_timeout_s", 1800.0))
        self.use_lidar = bool(p("use_lidar", True))
        self.z_band = (float(p("obstacle_min_z_m", 0.15)), float(p("obstacle_max_z_m", 1.7)))
        self.max_range = float(p("obstacle_max_range_m", 8.0))
        self.decay = float(p("obstacle_decay_s", 15.0))
        self.map_margin = float(p("lidar_map_margin_m", 0.25))
        instruction_topic = p("instruction_topic", "/vln/instruction")
        feedback_topic = p("feedback_topic", "/vln/feedback")

        self.lock = threading.Lock()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pose = MapPose(self, self.buffer, self.frame)
        self.pose = None
        self.loc_state = None
        self.task = None
        sub = lambda name, msg, cb, topic=None: self.create_subscription(msg, topic or name, cb, TOPICS[name][1])
        sub("/vln/instruction", String, self.on_instruction, instruction_topic)
        sub("/odom", Odometry, self.on_odom)
        sub("/localization/status", String, self.on_localization)
        sub("/nav/status", String, self.on_nav_status)
        if self.use_lidar:
            sub("/livox/lidar", PointCloud2, self.on_cloud)
        self.pub_plan = self.create_publisher(PathMsg, "/plan", TOPICS["/plan"][1])
        self.pub_feedback = self.create_publisher(String, feedback_topic, TOPICS["/vln/feedback"][1])
        self.pub_goal = self.create_publisher(PoseStamped, "/naive_pp/goal", LATCHED)
        self.pub_route = self.create_publisher(PathMsg, "/naive_pp/route", LATCHED)
        self.create_timer(0.5, self.tick)
        pkg = self.pp.pkg
        rooms = ", ".join(z["label"] for z in pkg.semantic["zones"])
        self.get_logger().info(f"map package {pkg.folder} ({pkg.grid.shape[1]}x{pkg.grid.shape[0]} @ "
                               f"{pkg.resolution} m): {len(pkg.places)} places, {len(self.pp.graph.edges)} doorways; "
                               f"rooms: {rooms}")

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg):
        self.pose = self.map_pose(msg) or self.pose

    def on_localization(self, msg):
        try:
            self.loc_state = json.loads(msg.data).get("state")
        except (ValueError, AttributeError):
            pass

    def localized(self):
        return self.pose is not None and (not self.need_fix or self.loc_state == "localized")

    def on_cloud(self, msg):
        if self.pose is None or self.task is None or self.task["state"] not in ("following", "retry"):
            return  # while a blocked robot waits to retry, keep seeing what blocks it
        try:
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        xyz = M.cloud_xyz(msg)
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng > 0.3) & (rng < self.max_range)]
        m = M.transform_matrix(tf.transform)
        world = xyz@m[:3, :3].T+m[:3, 3]
        world = world[(world[:, 2] > self.z_band[0]) & (world[:, 2] < self.z_band[1]), :2]
        cm = self.pp.costmap
        r, c = cm.world_to_cell(world)
        ok = cm.inside(r, c)
        world = world[ok][cm.static[r[ok], c[ok]] > self.map_margin]  # a mapped wall seen a little off is no obstacle
        with self.lock:
            cm.mark(world, self.now(), self.pose[:2])

    def on_instruction(self, msg):
        try:
            g = self.pp.ground(msg.data)
        except GroundingError as exc:
            self.feedback("failed", task_id=None, reason=str(exc), instruction=msg.data[:500])
            return
        if g.cancel:
            if self.task is not None and self.task["state"] not in ("done", "failed", "canceled"):
                self.hold()
                self.finish("canceled", reason="cancel requested")
            else:
                self.hold()
                self.feedback("canceled", task_id=g.task_id, reason="nothing to cancel; holding position")
            return
        if not g.ok:
            self.feedback("failed", task_id=g.task_id, reason=g.error, instruction=g.instruction[:500])
            return
        if self.task is not None and self.task["state"] not in ("done", "failed", "canceled"):
            self.finish("canceled", reason=f"preempted by task {g.task_id}")
        self.task = {"id": g.task_id, "g": g, "state": "waiting", "plan": None, "plan_stamp": None, "plan_id": None,
                     "replans": 0, "replanned": 0,
                     "started": self.now(), "last_plan": -math.inf, "progress": 0, "best_remaining": math.inf,
                     "progress_time": self.now(), "last_report": -math.inf, "travelled": 0.0, "last_xy": None}
        t = g.target
        self.feedback("accepted", instruction=g.instruction[:500], target={"kind": t.kind, "id": t.id, "label": t.label},
                      match=t.match, matched=t.matched, waypoints=[w.label for w in g.waypoints],
                      also_mentioned=g.alternatives)
        self.get_logger().info(f"task {g.task_id}: {t.label} ({t.match} '{t.matched}')"
                               + (f" via {', '.join(w.label for w in g.waypoints)}" if g.waypoints else ""))
        self.tick()

    def on_nav_status(self, msg):
        task = self.task
        if task is None or task["state"] != "following":
            return
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if status.get("source") != "follower" or status.get("goal_id") != task["plan_id"]:
            return
        if status.get("state") == "reached":
            gap = self.goal_gap()
            if gap is not None and gap <= self.goal_tolerance:
                self.finish("done")
            else:
                self.replan(f"follower reached the path end but the robot is {gap:.2f} m from the goal")
        elif status.get("state") == "failed":
            self.replan(f"follower failed: {status.get('message')}")

    # ------------------------------------------------------------------ task loop
    def tick(self):
        task = self.task
        if task is None or task["state"] in ("done", "failed", "canceled"):
            return
        now = self.now()
        if now-task["started"] > self.task_timeout:
            self.hold()
            self.finish("failed", reason=f"no arrival within {self.task_timeout:.0f} s")
            return
        if task["state"] == "waiting":
            if self.localized():
                self.plan("planned")
            elif now-task["last_report"] > 5.0:
                task["last_report"] = now
                self.feedback("waiting", reason="the robot is not localized in the map yet (no map -> odom, or the "
                              "localizer has no fix)", localization=self.loc_state)
            return
        if task["state"] == "retry":
            if now-task["last_plan"] >= self.replan_period:
                self.plan("replanned")
            return
        if task["state"] != "following" or self.pose is None:
            return
        if self.need_fix and self.loc_state == "lost":  # stand still until the localizer finds the robot again
            self.hold()
            task["state"] = "waiting"
            self.feedback("waiting", reason="localization lost; holding until the localizer has a fix again")
            return
        xy = np.array(self.pose[:2])
        if task["last_xy"] is not None:
            task["travelled"] += float(np.linalg.norm(xy-task["last_xy"]))
        task["last_xy"] = xy
        points = task["plan"].points
        lo = max(0, task["progress"]-5)
        hi = min(len(points), task["progress"]+120)
        d = np.linalg.norm(points[lo:hi]-xy, axis=1)
        task["progress"] = max(task["progress"], lo+int(np.argmin(d)))
        deviation = float(d.min())
        remaining = float(np.linalg.norm(np.diff(points[task["progress"]:], axis=0), axis=1).sum())+deviation
        if remaining < task["best_remaining"]-0.3:
            task["best_remaining"], task["progress_time"] = remaining, now
        with self.lock:
            self.pp.costmap.update(now, xy, decay_s=self.decay)
            blocked = self.pp.blocked(points, task["progress"])
        if blocked and now-task["last_plan"] >= self.replan_period:
            self.replan("an obstacle that is not on the map blocks the path", count=False)
        elif deviation > self.max_deviation and now-task["last_plan"] >= self.replan_period:
            self.replan(f"robot {deviation:.1f} m off the path", count=False)
        elif now-task["progress_time"] > self.stuck_timeout:
            self.replan(f"no progress for {self.stuck_timeout:.0f} s")
        elif now-task["last_report"] > 5.0:
            task["last_report"] = now
            self.feedback("progress", remaining_m=round(remaining, 1), place=self.place_label(xy),
                          pose=[round(v, 2) for v in self.pose], travelled_m=round(task["travelled"], 1),
                          lidar_obstacle_cells=self.pp.costmap.lidar_cells())

    def plan(self, event):
        task = self.task
        task["last_plan"] = self.now()
        g = task["g"]
        try:
            with self.lock:
                self.pp.costmap.update(self.now(), self.pose[:2], decay_s=self.decay)
                plan = self.pp.plan(self.pose, g.target, g.waypoints)
        except PlanningError as exc:
            task["replans"] += 1
            if task["replans"] > self.max_replans:
                self.hold()
                self.finish("failed", reason=str(exc))
            else:
                task["state"] = "retry"
                self.feedback("plan_failed", reason=str(exc), attempt=task["replans"])
                self.hold()
            return
        if task["plan_stamp"] is None:
            task["plan_stamp"] = M.stamp(self.now())
            task["plan_id"] = f"{task['plan_stamp'].sec}.{task['plan_stamp'].nanosec:09d}"
        msg = M.path_msg(self.now(), self.frame, plan.points, plan.final_yaw)
        msg.header.stamp = task["plan_stamp"]  # one plan id per task: the follower treats a new path as a replan
        for pose in msg.poses:
            pose.header.stamp = task["plan_stamp"]
        self.pub_plan.publish(msg)
        task.update(plan=plan, state="following", progress=0, best_remaining=plan.length_m, progress_time=self.now())
        task["replanned"] += event == "replanned"
        gx, gy, gyaw = plan.goal
        self.pub_goal.publish(M.pose_stamped(self.now(), self.frame, gx, gy, gyaw))
        anchors = [plan.start[:2], *(plan.route.doors if plan.route else []), plan.goal[:2]]
        self.pub_route.publish(M.path_msg(self.now(), self.frame, np.array(anchors, float), gyaw))
        self.feedback(event, **plan.summary(), start=[round(v, 2) for v in plan.start],
                      start_place=self.place_label(plan.start[:2]))
        self.get_logger().info(f"{event}: {plan.route.text() if plan.route else plan.target.label}; "
                               f"{plan.length_m:.1f} m ({plan.search}, {plan.planning_s:.2f} s)")

    def replan(self, reason, count=True):
        task = self.task
        if count:
            task["replans"] += 1
            if task["replans"] > self.max_replans:
                self.hold()
                self.finish("failed", reason=f"{reason}; gave up after {self.max_replans} replans")
                return
        self.get_logger().info(f"replanning: {reason}")
        self.feedback("replanning", reason=reason)
        self.plan("replanned")

    def hold(self):
        """Stop where the robot stands: a one-pose path (the follower keeps that pose)."""
        if self.pose is None:
            return
        x, y, yaw = self.pose
        self.pub_plan.publish(M.path_msg(self.now(), self.frame, np.array([[x, y]]), yaw))

    def goal_gap(self):
        if self.pose is None or self.task is None or self.task["plan"] is None:
            return None
        gx, gy, _ = self.task["plan"].goal
        return math.hypot(self.pose[0]-gx, self.pose[1]-gy)

    def place_label(self, xy):
        pid = self.pp.pkg.place_at(xy)
        return self.pp.graph.label(pid) if pid else None

    def finish(self, state, **fields):
        task = self.task
        task["state"] = state
        extra = {}
        if self.pose is not None:
            x, y, yaw = self.pose
            pid = self.pp.pkg.place_at((x, y))
            t = task["g"].target
            extra = {"pose": [round(x, 3), round(y, 3), round(yaw, 3)], "place": self.place_label((x, y)),
                     "in_target_place": (pid == (t.id if t.kind == "place" else t.place)) if pid else False}
            gap = self.goal_gap()
            if gap is not None:
                extra["goal_distance_m"] = round(gap, 3)
        extra.update(elapsed_s=round(self.now()-task["started"], 1), travelled_m=round(task["travelled"], 1),
                     replans=task["replanned"])
        if task["plan"] is not None:
            extra["path_length_m"] = round(task["plan"].length_m, 2)
        self.feedback(state, **extra, **fields)
        self.get_logger().info(f"task {task['id']} {state}: {json.dumps({**extra, **fields}, ensure_ascii=False)}")

    def feedback(self, state, task_id="__current__", **fields):
        if task_id == "__current__":
            task_id = self.task["id"] if self.task else None
        self.pub_feedback.publish(String(data=json.dumps({"task_id": task_id, "state": state,
                                                          "sim_time_s": round(self.now(), 3), **fields},
                                                         ensure_ascii=False)))


def main():
    rclpy.init()
    node = NaivePPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
