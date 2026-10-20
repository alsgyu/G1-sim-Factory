"""/global_planner: layer 2 - A* on /map plus a LiDAR obstacle layer -> /plan.

Replans when the active path becomes blocked by obstacles that are not in the
static map (seen by /livox/lidar), and reports failures on /nav/status.
"""
from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_factory.planning import CostMap, OccupancyGrid as Grid, PlanningError, path_blocked, plan_path
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import TOPICS


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner")
        p = self.declare_parameter
        self.lethal = float(p("lethal_radius_m", 0.40).value)
        self.obstacle_z = (float(p("obstacle_min_z_m", 0.15).value), float(p("obstacle_max_z_m", 1.70).value))
        self.max_range = float(p("obstacle_max_range_m", 8.0).value)
        self.decay = float(p("obstacle_decay_s", 15.0).value)
        self.costmap = None
        self.pose = None
        self.goal = None          # (x, y, yaw, goal_id)
        self.goal_stamp = None    # builtin_interfaces/Time of the goal = plan id (copied, never rounded)
        self.path = None
        self.progress = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pose = MapPose(self, self.buffer)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        sub("/map", OccupancyGrid, self.on_map)
        sub("/odom", Odometry, self.on_odom)
        sub("/goal_pose", PoseStamped, self.on_goal)
        sub("/livox/lidar", PointCloud2, self.on_cloud)
        self.pub_plan = self.create_publisher(Path, "/plan", TOPICS["/plan"][1])
        self.pub_status = self.create_publisher(String, "/nav/status", TOPICS["/nav/status"][1])
        self.create_timer(0.5, self.check_path)
        self.marked = 0

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_map(self, msg: OccupancyGrid):
        grid = Grid((msg.info.origin.position.x, msg.info.origin.position.y), msg.info.resolution,
                    M.occupancy_to_numpy(msg))
        self.costmap = CostMap(grid, lethal_radius=self.lethal)
        self.get_logger().info(f"Static map {grid.width}x{grid.height} @ {grid.resolution} m")

    def on_odom(self, msg: Odometry):
        self.pose = self.map_pose(msg) or self.pose

    def on_cloud(self, msg: PointCloud2):
        if self.costmap is None:
            return
        try:  # non-blocking: never stall the executor waiting for TF
            tf = self.buffer.lookup_transform("map", msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return  # no transform for this scan stamp (yet); skip the scan
        xyz = M.cloud_xyz(msg)
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng > 0.3) & (rng < self.max_range)]
        matrix = M.transform_matrix(tf.transform)
        world = xyz@matrix[:3, :3].T+matrix[:3, 3]
        world = world[(world[:, 2] > self.obstacle_z[0]) & (world[:, 2] < self.obstacle_z[1])]
        self.marked = self.costmap.mark_points(world[:, :2], self.now())

    def on_goal(self, msg: PoseStamped):
        goal_id = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        if msg.header.frame_id != "map":
            self.report("plan_failed", goal_id, f"goal frame {msg.header.frame_id!r} must be 'map'")
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y, M.quaternion_yaw(msg.pose.orientation), goal_id)
        self.goal_stamp = msg.header.stamp
        self.plan("new_goal")

    def plan(self, reason):
        if self.costmap is None or self.pose is None or self.goal is None:
            self.report("plan_failed", self.goal[3] if self.goal else None, "map or odometry not received yet")
            return
        if self.pose is not None:
            self.costmap.update(self.now(), self.pose[:2], decay_s=self.decay)
        x, y, yaw, goal_id = self.goal
        try:
            points, info = plan_path(self.costmap, self.pose[:2], (x, y))
        except PlanningError as exc:
            self.path = None
            self.report("plan_failed", goal_id, str(exc))
            return
        self.path, self.progress = points, 0
        msg = M.path_msg(self.now(), "map", points, yaw)
        msg.header.stamp = self.goal_stamp  # plan id = goal stamp, so status can be matched to the goal
        for pose in msg.poses:
            pose.header.stamp = self.goal_stamp
        self.pub_plan.publish(msg)
        snapped = float(np.hypot(points[-1, 0]-x, points[-1, 1]-y))
        self.report("planned", goal_id, reason, length_m=round(info["length_m"], 2), poses=len(points),
                    vertices=info["vertices"], expansions=info["expansions"], goal_snap_m=round(snapped, 2),
                    lidar_cells=int((self.costmap.dynamic_hits > 0).sum()))

    def check_path(self):
        """Replan when unmapped obstacles block the remaining path."""
        if self.path is None or self.pose is None or self.costmap is None:
            return
        xy = np.asarray(self.pose[:2])
        if np.hypot(*(self.path[-1]-xy)) < 0.3:
            self.path = None  # follower owns the final approach and alignment
            return
        self.costmap.update(self.now(), xy, decay_s=self.decay)
        dist = np.linalg.norm(self.path-xy, axis=1)
        self.progress = max(self.progress, int(np.argmin(dist)))
        if path_blocked(self.costmap, self.path, self.progress):
            self.get_logger().info("Path blocked by an obstacle not in /map; replanning")
            self.plan("replan_blocked")

    def report(self, state, goal_id, message="", **extra):
        self.pub_status.publish(String(data=json.dumps({"source": "planner", "state": state, "goal_id": goal_id,
                                                        "message": message, "sim_time_s": round(self.now(), 3),
                                                        **extra})))


def main():
    rclpy.init()
    node = GlobalPlannerNode()
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
