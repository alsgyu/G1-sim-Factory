"""naive_pp_client: stands in for the external agent in tests: sends one instruction, records the run, exits.

    python3 -m naive_pp.ros.client_node --ros-args -p instruction:="Go to the pump room" -p output_dir:=/tmp/run

Waits until the localizer has a fix (so the instruction is not queued behind localization), sends the instruction on
/vln/instruction once, prints /vln/feedback, and writes to output_dir:
  feedback.jsonl     every feedback event
  trajectory.csv     t, map x/y/yaw (/localization/pose), odom x/y/yaw (/odom; in the simulator without odom drift
                     this is the true pose, used only to evaluate)
  plan.csv           the last /plan
  summary.json       instruction, terminal state, final pose, place, durations
Exits when the task is done, failed or canceled, or after timeout_s (simulation time).
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String

from naive_pp import SIM  # noqa: F401  (puts g1_factory on the path)
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS


class ClientNode(Node):
    def __init__(self):
        super().__init__("naive_pp_client")
        p = lambda name, default: self.declare_parameter(name, default).value
        self.instruction = p("instruction", "")
        path = p("instruction_file", "")
        if path:  # any text, quotes included, without ROS parameter parsing
            self.instruction = Path(path).read_text(encoding="utf-8").strip()
        self.out = Path(p("output_dir", "/tmp/naive_pp_run"))
        self.timeout = float(p("timeout_s", 1800.0))
        self.wait_fix = bool(p("wait_for_localization", True))
        self.settle = float(p("settle_s", 3.0))
        if not self.instruction:
            raise SystemExit("naive_pp_client needs instruction:=... or instruction_file:=...")
        self.out.mkdir(parents=True, exist_ok=True)
        self.events = (self.out/"feedback.jsonl").open("w", encoding="utf-8")
        self.traj_file = (self.out/"trajectory.csv").open("w", newline="")
        self.traj = csv.writer(self.traj_file)
        self.traj.writerow(["t", "map_x", "map_y", "map_yaw", "odom_x", "odom_y", "odom_yaw"])
        self.map_pose = None
        self.odom_pose = None
        self.fix_time = None
        self.sent_time = None
        self.final = None
        self.last_row = -1.0
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        sub("/vln/feedback", String, self.on_feedback)
        sub("/localization/status", String, self.on_localization)
        sub("/localization/pose", PoseWithCovarianceStamped, self.on_map_pose)
        sub("/odom", Odometry, self.on_odom)
        sub("/plan", PathMsg, self.on_plan)
        self.pub = self.create_publisher(String, "/vln/instruction", TOPICS["/vln/instruction"][1])
        self.create_timer(0.2, self.tick)
        self.start = None

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_localization(self, msg):
        state = json.loads(msg.data).get("state")
        if state == "localized" and self.fix_time is None:
            self.fix_time = self.now()
            self.get_logger().info("localizer has a fix")

    def on_map_pose(self, msg):
        p = msg.pose.pose
        self.map_pose = (p.position.x, p.position.y, M.quaternion_yaw(p.orientation))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, M.quaternion_yaw(p.orientation))
        t = self.now()
        if self.sent_time is not None and t-self.last_row >= 0.2:
            self.last_row = t
            self.traj.writerow([round(t, 2), *(round(v, 4) for v in (self.map_pose or (float("nan"),)*3)),
                                *(round(v, 4) for v in self.odom_pose)])

    def on_plan(self, msg):
        points, yaw = M.path_to_numpy(msg)
        with (self.out/"plan.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y"])
            w.writerows(points.round(3).tolist())

    def on_feedback(self, msg):
        event = json.loads(msg.data)
        self.events.write(json.dumps(event, ensure_ascii=False)+"\n")
        self.events.flush()
        brief = {k: v for k, v in event.items() if k not in ("task_id", "sim_time_s", "doors")}
        self.get_logger().info(f"feedback: {json.dumps(brief, ensure_ascii=False)[:400]}")
        if event.get("state") in ("done", "failed", "canceled") and self.sent_time is not None:
            self.final = event

    def tick(self):
        now = self.now()
        if self.start is None:
            self.start = now
        if self.sent_time is None:
            ready = self.pub.get_subscription_count() > 0 and (not self.wait_fix or (
                self.fix_time is not None and now-self.fix_time >= self.settle))
            if ready:
                self.pub.publish(String(data=self.instruction))
                self.sent_time = now
                self.get_logger().info(f"instruction sent: {self.instruction}")
            elif now-self.start > self.timeout:
                self.done({"state": "failed", "reason": "never localized / no subscriber"})
            return
        if self.final is not None:
            self.done(self.final)
        elif now-self.sent_time > self.timeout:
            self.done({"state": "failed", "reason": f"client timeout {self.timeout:.0f} s"})

    def done(self, final):
        summary = {"instruction": self.instruction, "state": final.get("state"), "final_event": final,
                   "sent_sim_time_s": self.sent_time, "localized_sim_time_s": self.fix_time,
                   "duration_s": round(self.now()-self.sent_time, 1) if self.sent_time else None,
                   "final_map_pose": self.map_pose, "final_odom_pose": self.odom_pose}
        (self.out/"summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False)+"\n", encoding="utf-8")
        self.get_logger().info(f"finished: {final.get('state')} -> {self.out/'summary.json'}")
        raise SystemExit(0)


def main():
    rclpy.init()
    node = ClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.events.close()
        node.traj_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
