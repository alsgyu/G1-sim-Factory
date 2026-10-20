"""/path_follower: layer 3 - pure pursuit + lateral correction on /plan -> /cmd_vel (20 Hz).

Body-frame [vx, vy, yaw_rate] goes to the G1 walking policy (layer 4). Reports
reached/failed for the plan id (= goal stamp) on /nav/status.
"""
from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String

from g1_factory.planning import PathTracker
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import TOPICS


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__("path_follower")
        p = self.declare_parameter
        self.tracker = PathTracker(max_speed=float(p("max_speed_mps", 0.40).value),
                                   max_lateral=float(p("max_lateral_mps", 0.15).value),
                                   max_yaw_rate=float(p("max_yaw_rate_rps", 0.60).value),
                                   lookahead=float(p("lookahead_m", 0.60).value),
                                   lateral_gain=float(p("lateral_gain", 0.8).value),
                                   goal_tolerance=float(p("goal_tolerance_m", 0.15).value),
                                   yaw_tolerance=float(p("yaw_tolerance_rad", 0.10).value))
        self.rate = float(p("rate_hz", 20.0).value)
        self.timeout = float(p("goal_timeout_s", 240.0).value)
        self.odom_timeout = float(p("odom_timeout_s", 0.5).value)
        self.pose, self.odom_time = None, -math.inf
        self.map_pose = MapPose(self)  # plans are in the map frame; /odom is in odom
        self.plan_id, self.plan_start = None, 0.0
        self.errors = []
        self.create_subscription(Path, "/plan", self.on_plan, TOPICS["/plan"][1])
        self.create_subscription(Odometry, "/odom", self.on_odom, TOPICS["/odom"][1])
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", TOPICS["/cmd_vel"][1])
        self.pub_status = self.create_publisher(String, "/nav/status", TOPICS["/nav/status"][1])
        self.create_timer(1.0/self.rate, self.tick)
        self.last_report = 0.0
        self.idle_ticks = 0

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self, msg: Odometry):
        pose = self.map_pose(msg)
        if pose is None:  # no map -> odom (not localised): treated like missing odometry, the robot stops
            return
        self.pose, self.odom_time = pose, self.now()

    def on_plan(self, msg: Path):
        points, final_yaw = M.path_to_numpy(msg)
        if len(points) == 0:
            self.tracker.clear()
            return
        plan_id = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        replan = plan_id == self.plan_id
        self.tracker.set_path(points, final_yaw)
        if not replan:
            self.plan_id, self.plan_start, self.errors = plan_id, self.now(), []

    def tick(self):
        cmd = np.zeros(3, dtype=np.float32)
        now = self.now()
        active = self.tracker.state in ("following", "aligning", "reached")
        if self.pose is not None and now-self.odom_time <= self.odom_timeout and active:
            x, y, yaw = self.pose
            was = self.tracker.state
            cmd = self.tracker.command((x, y), yaw)
            if self.tracker.state == "following":
                self.errors.append(abs(self.tracker.cross_track((x, y))))
            if self.tracker.state == "reached":
                if was != "reached":  # report once; then hold the pose until the next plan
                    self.report("reached", x=round(x, 3), y=round(y, 3), yaw=round(yaw, 3))
            elif now-self.plan_start > self.timeout:
                self.report("failed", message="goal timeout")
                self.tracker.clear()
                cmd[:] = 0
            elif now-self.last_report > 1.0:
                self.last_report = now
                remaining = float(np.linalg.norm(self.tracker.path[-1]-np.array([x, y])))
                self.report("following", remaining_m=round(remaining, 2),
                            cross_track_m=round(self.errors[-1], 3) if self.errors else None)
        # Publish while a plan is active (incl. pose hold), then a few zeros; stay silent when
        # idle so teleoperation or another controller can own /cmd_vel.
        if self.tracker.state != "idle":
            self.idle_ticks = 0
        else:
            self.idle_ticks += 1
            if self.idle_ticks > 3:
                return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        self.pub_cmd.publish(msg)

    def report(self, state, message="", **extra):
        errors = np.asarray(self.errors) if self.errors else None
        stats = {} if errors is None else {"cross_track_rmse_m": round(float(np.sqrt(np.mean(errors**2))), 3),
                                           "cross_track_max_m": round(float(errors.max()), 3)}
        self.pub_status.publish(String(data=json.dumps({"source": "follower", "state": state, "goal_id": self.plan_id,
                                                        "message": message, "sim_time_s": round(self.now(), 3),
                                                        "elapsed_s": round(self.now()-self.plan_start, 2),
                                                        **stats, **extra})))


def main():
    rclpy.init()
    node = PathFollowerNode()
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
