"""mapping_tour: walks the G1 through the campus once for SLAM mapping (in place of a person with a joystick).

Stands still for `settle_s` (FAST-LIO2 initialises and the map node measures gravity) and until slam_map reports
that scans come in (gives up after `slam_timeout_s` instead of walking a tour that maps nothing; wait_for_slam:=false
walks a route without SLAM and stays until the launch ends, launch/g1_localization_eval.launch.py), then sends the
waypoints of
the tour (configs/slam_mapping.yaml) one by one on /goal_pose to global_planner + path_follower. A waypoint ends
with reached or failed on /nav/status; failures are logged and skipped. At the end it publishes /mapping/done,
waits until slam_map reports the map saved on /slam/status, and exits (the mapping launch then shuts down).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]


class MappingTour(Node):
    def __init__(self):
        super().__init__("mapping_tour")
        p = self.declare_parameter
        cfg = yaml.safe_load(Path(p("tour_file", str(ROOT/"configs/slam_mapping.yaml")).value).read_text())
        self.waypoints = [tuple(w) for w in cfg["tours"][p("tour", "campus").value]["waypoints"]]
        self.settle = float(p("settle_s", 6.0).value)
        self.slam_timeout = float(p("slam_timeout_s", 90.0).value)
        self.slam_ready, self.last_wait_log, self.slam_message = False, 0.0, ""
        self.save_timeout = float(p("save_timeout_s", 120.0).value)
        # false: walk a route without SLAM (localisation measurement, launch/g1_localization_eval.launch.py)
        self.wait_for_slam = bool(p("wait_for_slam", True).value)
        self.slam_ready = not self.wait_for_slam
        self.pose, self.index, self.goal_id, self.t0 = None, -1, None, None
        self.done_at, self.finished = None, False
        self.log = []
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", TOPICS["/goal_pose"][1])
        self.pub_done = self.create_publisher(String, "/mapping/done", TOPICS["/vln/instruction"][1])
        self.create_subscription(Odometry, "/odom", self.on_odom, TOPICS["/odom"][1])
        self.create_subscription(String, "/nav/status", self.on_status, TOPICS["/nav/status"][1])
        self.create_subscription(String, "/slam/status", self.on_slam, TOPICS["/nav/status"][1])
        self.create_timer(0.5, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def tick(self):
        if self.pose is None or self.finished:
            return
        if self.t0 is None:
            self.t0 = self.now()
            self.get_logger().info(f"standing {self.settle:.0f} s{' for SLAM initialisation' if self.wait_for_slam else ''}, "
                                   f"then {len(self.waypoints)} waypoints")
        waited = self.now()-self.t0
        if self.index < 0 and waited >= self.settle:
            if self.slam_ready:
                self.next_goal()
            elif waited >= self.settle+self.slam_timeout:
                self.get_logger().error(f"SLAM did not start in {self.slam_timeout:.0f} s, not walking the tour: "
                                        f"{self.slam_message or 'no status from slam_map'}")
                self.finished = True
            elif waited-self.last_wait_log >= 10.0:
                self.last_wait_log = waited
                self.get_logger().info(f"waiting for SLAM scans ({waited:.0f} s) {self.slam_message}")
        if self.done_at is not None and self.wait_for_slam and self.now()-self.done_at > self.save_timeout:
            self.get_logger().error("map node did not report the map saved; exiting")
            self.finished = True

    def next_goal(self, outcome=None):
        if outcome:
            self.log.append({"waypoint": self.waypoints[self.index], "outcome": outcome})
            self.get_logger().info(f"waypoint {self.index+1}/{len(self.waypoints)} {outcome}")
        self.index += 1
        if self.index >= len(self.waypoints):
            failed = sum(1 for e in self.log if e["outcome"] != "reached")
            self.get_logger().info(f"tour finished ({failed} waypoints not reached)"
                                   + ("; saving the map" if self.wait_for_slam else ""))
            self.pub_done.publish(String(data=json.dumps({"waypoints": self.log})))
            self.done_at = self.now()
            return
        x, y = self.waypoints[self.index]
        nx, ny = self.waypoints[self.index+1] if self.index+1 < len(self.waypoints) else (x+1, y)
        msg = M.pose_stamped(self.now(), "map", x, y, math.atan2(ny-y, nx-x))
        self.goal_id = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        self.pub_goal.publish(msg)

    def on_status(self, msg):
        s = json.loads(msg.data)
        if s.get("goal_id") != self.goal_id or self.done_at is not None:
            return
        if s.get("source") == "follower" and s.get("state") in ("reached", "failed"):
            self.next_goal(s["state"])
        elif s.get("source") == "planner" and s.get("state") == "plan_failed":
            self.next_goal("plan_failed: "+s.get("message", ""))

    def on_slam(self, msg):
        s = json.loads(msg.data)
        if s.get("state") in ("tracking", "mapping"):
            self.slam_ready = True
        elif s.get("state") == "waiting":
            self.slam_message = s.get("message", "")
        if s.get("state") == "saved" and self.done_at is not None:
            self.get_logger().info(f"map saved: {s.get('folder')}")
            self.finished = True


def main():
    rclpy.init()
    node = MappingTour()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
