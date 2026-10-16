"""/external_agent: stand-in for the upstream agent that commands the robot, plus evaluation.

Sends one instruction from a task file on /vln/instruction, records /vln/feedback,
the inspection image and the trajectory, then writes summary.json and exits (the
launch file shuts the graph down on exit). A real external agent only needs the
/vln/instruction + /vln/feedback (+ /vln/inspection_image) contract; see
scripts/send_instruction.py.

Metrics (per episode): success, grounding correctness, subgoals reached, final
distance to the expected place, SPL against the A* shortest path on /map,
collisions/falls, cross-track RMSE, inspection verdict vs. the expected verdict,
VLM latency. A wall-clock watchdog ends the episode if simulation time stalls.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from g1_factory.grounding import SemanticMap
from g1_factory.planning import CostMap, OccupancyGrid as Grid, PlanningError, plan_path
from g1_factory.scene import load_config
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TASKS = ROOT/"configs/vln_tasks.yaml"


def load_task(name: str, path=DEFAULT_TASKS) -> dict:
    tasks = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["tasks"]
    if name not in tasks:
        raise SystemExit(f"Unknown task {name}; choose {sorted(tasks)}")
    return tasks[name]


class ExternalAgentNode(Node):
    def __init__(self):
        super().__init__("external_agent")
        p = self.declare_parameter
        self.task_name = p("task", "inspect_conveyor").value
        self.task = load_task(self.task_name, p("tasks_file", str(DEFAULT_TASKS)).value)
        self.out = Path(p("output_dir", str(ROOT/"outputs/ros2_run")).value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.timeout = float(p("timeout_s", 900.0).value)                   # simulation seconds
        self.stall_timeout = float(p("stall_timeout_wall_s", 120.0).value)  # wall seconds without /clock progress
        self.send_after = float(p("send_after_s", 3.0).value)
        self.success_radius = float(p("success_radius_m", 1.5).value)
        self.condition = p("condition", "").value
        self.seed = int(p("seed", 0).value)
        smap = SemanticMap.load(load_config(ROOT/"configs/factory.yaml"))
        self.expected = smap.places[self.task["expected_place"]]
        instruction = self.task["instruction"]
        self.payload = instruction if isinstance(instruction, str) else json.dumps(instruction, ensure_ascii=False)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        self.pub = self.create_publisher(String, "/vln/instruction", TOPICS["/vln/instruction"][1])
        sub("/vln/feedback", String, self.on_feedback)
        sub("/vln/inspection_image", CompressedImage, self.on_image)
        sub("/odom", Odometry, self.on_odom)
        sub("/sim/status", String, self.on_sim)
        sub("/nav/status", String, self.on_nav)
        sub("/plan", PathMsg, self.on_plan)
        sub("/map", OccupancyGrid, self.on_map)
        self.sent_at, self.finished = None, False
        self.have_image, self.pending_done, self.pending_since = False, None, 0.0
        self.feedback, self.nav, self.plans, self.trajectory = [], [], [], []
        self.sim, self.pose, self.start_pose, self.last_traj = {}, None, None, -math.inf
        self.grid = None
        self.last_sim, self.last_progress_wall = -1.0, time.monotonic()
        self.feedback_file = (self.out/"feedback.jsonl").open("w", encoding="utf-8")
        self.create_timer(0.5, self.tick)  # simulation time
        self.create_timer(2.0, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ timers
    def tick(self):
        t = self.now()
        if self.sent_at is None and t >= self.send_after and self.pub.get_subscription_count() > 0 and self.pose:
            self.start_pose = self.pose
            self.pub.publish(String(data=self.payload))
            self.sent_at = t
            self.get_logger().info(f"Instruction sent: {self.payload}")
        if self.pending_done is not None and not self.finished and t-self.pending_since > 2.0:
            self.finish(self.pending_done)
        if self.sent_at is not None and not self.finished and t-self.sent_at > self.timeout:
            self.finish("timeout")

    def watchdog(self):
        """Simulation time stops when g1_sim dies; sim-time timers would then never fire."""
        t = self.now()
        if t > self.last_sim:
            self.last_sim, self.last_progress_wall = t, time.monotonic()
        elif not self.finished and time.monotonic()-self.last_progress_wall > self.stall_timeout:
            self.finish("sim_stalled")

    # --------------------------------------------------------------- callbacks
    def on_map(self, msg):
        self.grid = Grid((msg.info.origin.position.x, msg.info.origin.position.y), msg.info.resolution,
                         M.occupancy_to_numpy(msg))

    def on_odom(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, M.quaternion_yaw(msg.pose.pose.orientation))
        t = M.to_seconds(msg.header.stamp)
        if t-self.last_traj >= 0.2:
            self.last_traj = t
            self.trajectory.append([round(t, 3), *(round(v, 4) for v in self.pose)])

    def on_sim(self, msg):
        self.sim = json.loads(msg.data)
        if self.sim.get("fallen") and self.sent_at is not None and not self.finished:
            self.finish("fallen")

    def on_nav(self, msg):
        self.nav.append(json.loads(msg.data))

    def on_plan(self, msg):
        pts, _ = M.path_to_numpy(msg)
        length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0
        self.plans.append({"sim_time_s": round(self.now(), 3), "poses": len(pts), "length_m": round(length, 2),
                           "points": pts[::5].round(2).tolist()+[pts[-1].round(2).tolist()]})

    def on_image(self, msg):
        (self.out/"inspection.jpg").write_bytes(bytes(msg.data))
        self.have_image = True
        if self.pending_done is not None and not self.finished:
            self.finish(self.pending_done)

    def on_feedback(self, msg):
        event = json.loads(msg.data)
        self.feedback.append(event)
        self.feedback_file.write(json.dumps(event, ensure_ascii=False)+"\n")
        self.feedback_file.flush()
        brief = {k: v for k, v in event.items()
                 if k in ("place", "subgoals", "target", "question", "result", "message", "distance_m")}
        self.get_logger().info(f"feedback: {event.get('state')} {json.dumps(brief, ensure_ascii=False)}")
        if event.get("state") == "done" and not self.finished and not self.have_image:
            # The evidence image travels on its own topic and may arrive just after "done".
            self.pending_done, self.pending_since = "done", self.now()
        elif event.get("state") in ("done", "failed") and not self.finished:
            self.finish(event["state"])

    # -------------------------------------------------------------- evaluation
    def shortest_path(self):
        if self.grid is None or self.start_pose is None:
            return None
        try:
            _, info = plan_path(CostMap(self.grid), self.start_pose[:2], self.expected.pose[:2])
            return info["length_m"]
        except PlanningError:
            return None

    def finish(self, state):
        if self.finished:
            return
        self.finished = True
        planned = next((e for e in self.feedback if e.get("state") == "planned"), {})
        done = next((e for e in self.feedback if e.get("state") == "done"), {})
        reached = [e for e in self.feedback if e.get("state") == "subgoal_reached"]
        subgoals = planned.get("subgoals", [])
        x, y, _ = self.pose if self.pose else (math.nan, math.nan, 0)
        ex, ey, _ = self.expected.pose
        final_error = float(math.hypot(x-ex, y-ey))
        followers = [e for e in self.nav if e.get("source") == "follower" and e.get("state") == "reached"]
        rmse = [e["cross_track_rmse_m"] for e in followers if e.get("cross_track_rmse_m") is not None]
        inspection = done.get("result") or {}
        expected_verdict = self.task.get("expected_verdict")
        calls = done.get("vlm_calls") or next((e.get("vlm_calls") for e in self.feedback if e.get("state") == "failed"), [])
        latencies = [c["latency_s"] for c in calls if c.get("latency_s")]
        travelled = self.sim.get("travelled_m") or 0.0
        shortest = self.shortest_path()
        summary = {
            "task": self.task_name, "condition": self.condition, "seed": self.seed, "instruction": self.payload,
            "state": state, "expected_place": self.expected.id, "planned_subgoals": subgoals,
            "parser": planned.get("parser"),
            "grounding_correct": bool(subgoals) and subgoals[-1] == self.expected.id,
            "subgoals_reached": len(reached), "navigation_complete": bool(subgoals) and len(reached) == len(subgoals),
            "start_pose": self.start_pose, "final_pose": self.pose,
            "final_distance_to_expected_place_m": round(final_error, 3),
            "target_xy": done.get("target_xy"), "inspection": inspection or None,
            "expected_verdict": expected_verdict,
            "verdict_correct": (inspection.get("verdict") == expected_verdict) if expected_verdict and inspection else None,
            "collision_events": self.sim.get("collision_events"), "collision_geoms": self.sim.get("collision_geoms"),
            "fallen": bool(self.sim.get("fallen")), "travelled_m": travelled, "shortest_path_m": shortest,
            "task_sim_time_s": round(self.now()-(self.sent_at or 0), 2),
            "plans": len(self.plans), "replans": sum(1 for e in self.nav if e.get("message") == "replan_blocked"),
            "cross_track_rmse_m_per_goal": rmse, "vlm_calls": calls,
            "vlm_latency_mean_s": round(float(np.mean(latencies)), 3) if latencies else None,
            "realtime_factor": self.sim.get("realtime_factor"), "sim_timing_ms_mean": self.sim.get("timing_ms_mean"),
        }
        success = bool(state == "done" and summary["grounding_correct"] and summary["navigation_complete"]
                       and final_error <= self.success_radius and not summary["collision_events"]
                       and not summary["fallen"] and inspection.get("answer"))
        summary["success"] = success
        summary["spl"] = round(shortest/max(shortest, travelled), 3) if success and shortest and travelled else 0.0
        (self.out/"summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
        (self.out/"plans.json").write_text(json.dumps(self.plans)+"\n")
        (self.out/"nav_status.jsonl").write_text("".join(json.dumps(e)+"\n" for e in self.nav))
        with (self.out/"trajectory.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sim_time_s", "x_m", "y_m", "yaw_rad"])
            writer.writerows(self.trajectory)
        self.get_logger().info(f"Task finished ({state}); success={success}; results in {self.out}")
        raise SystemExit(0 if success else 1)


def main():
    rclpy.init()
    node = ExternalAgentNode()
    code = 1
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    except KeyboardInterrupt:
        pass
    finally:
        node.feedback_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
