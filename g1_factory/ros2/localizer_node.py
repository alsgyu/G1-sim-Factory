"""localizer: the robot's pose in a saved map package (TF map -> odom), for the VLN with package_frame:=map.

    python3 -m g1_factory.ros2.localizer_node --ros-args -p package:=maps/campus -p initial_pose:="0,0,0"

A robot that comes back to its map does not know where it stands in it. Starting from a rough pose a person gives
(initial_pose, or RViz "2D Pose Estimate" on /initialpose; within 1.5 m and 20 deg), this node matches LiDAR scans
against the package's occupancy grid (g1_factory.localization) and keeps the estimate while the robot walks.

Subscribes
  /livox/lidar        LiDAR scans (any PointCloud2), moved into odom with TF at the scan stamp
  /odom               odometry in a gravity-aligned odom frame (z up): the simulator's, the G1's or levelled FAST-LIO2
  /initialpose        geometry_msgs/PoseWithCovarianceStamped, map frame: (re)start from this rough pose
  /tf, /tf_static     odom -> base_link -> LiDAR frame
Publishes
  /tf                    map -> odom at 20 Hz, stamped transform_tolerance_s ahead like AMCL, so lookups at sensor
                         stamps never wait for the next match; nothing before the first fix (the VLN waits)
  /localization/pose     PoseWithCovarianceStamped of base_link in the map frame, 10 Hz
  /localization/status   JSON per match: state (waiting, initializing, localized, lost), score, inlier share,
                         correction, latency, map_from_odom
"""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from g1_factory import slam_map as S
from g1_factory.localization import Localizer, compose
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS


def parse_pose(value) -> tuple | None:
    """'x,y,yaw' (or a list) -> (x, y, yaw); empty -> None."""
    if isinstance(value, str):
        value = [v for v in value.replace(",", " ").replace("[", " ").replace("]", " ").split()]
    if value is None or len(value) == 0:
        return None
    if len(value) != 3:
        raise ValueError("initial_pose must be x,y,yaw (map frame, m and rad)")
    return tuple(float(v) for v in value)


def odom_pose(msg: Odometry):
    p = msg.pose.pose.position
    return p.x, p.y, M.quaternion_yaw(msg.pose.pose.orientation)


class LocalizerNode(Node):
    def __init__(self):
        super().__init__("localizer")
        p = self.declare_parameter
        package = p("package", "").value
        if not package:
            raise SystemExit("localizer needs package:=maps/<name>")
        self.package = Path(package)
        grid, origin, res = S.read_map_yaml(self.package/"map.yaml")
        self.loc = Localizer(grid, origin, res,
                             init_window=(float(p("init_window_m", 1.5).value),
                                          math.radians(float(p("init_window_deg", 20.0).value))),
                             track_window=(float(p("track_window_m", 0.4).value),
                                           math.radians(float(p("track_window_deg", 5.0).value))),
                             min_inliers=float(p("min_inliers", 0.5).value),
                             init_min_inliers=float(p("init_min_inliers", 0.8).value),
                             max_points=int(p("max_points", 800).value))
        self.map_frame = p("map_frame", "map").value
        # false: measurement only (scripts/eval: the robot is driven on the simulator's map, map -> odom stays its own)
        self.tf_enabled = bool(p("publish_tf", True).value)
        self.period = 1.0/float(p("rate_hz", 2.0).value)
        self.tolerance = float(p("transform_tolerance_s", 0.5).value)
        self.pending = parse_pose(p("initial_pose", "", ParameterDescriptor(dynamic_typing=True)).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        sub("/livox/lidar", PointCloud2, self.on_cloud)
        sub("/odom", Odometry, self.on_odom)
        sub("/initialpose", PoseWithCovarianceStamped, self.on_initial_pose)
        pub = lambda name, msg: self.create_publisher(msg, name, TOPICS[name][1])
        self.pub_pose = pub("/localization/pose", PoseWithCovarianceStamped)
        self.pub_status = pub("/localization/status", String)
        self.scans = deque(maxlen=5)
        self.odom = None
        self.last_scan_stamp = None
        self.pool = ThreadPoolExecutor(1)
        self.job = None
        self.state, self.cov, self.last_match = "waiting", np.diag([0.25, 0.25, 0.07]), -math.inf
        self.create_timer(self.period/2, self.tick)
        self.create_timer(0.05, self.publish_tf)
        self.create_timer(0.1, self.publish_pose)
        self.get_logger().info(f"map {self.package} ({grid.shape[1]}x{grid.shape[0]} @ {res} m); "
                               + (f"initial pose {self.pending}" if self.pending else "waiting for /initialpose"))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ inputs
    def on_cloud(self, msg):
        self.scans.append(msg)

    def on_odom(self, msg):
        self.odom = msg

    def on_initial_pose(self, msg):
        if msg.header.frame_id not in ("", self.map_frame):
            self.get_logger().warn(f"/initialpose in frame {msg.header.frame_id}, expected {self.map_frame}")
            return
        self.pending = odom_pose(msg)
        self.get_logger().info(f"initial pose from /initialpose: {[round(v, 2) for v in self.pending]}")

    # ------------------------------------------------------------------ matching
    def scan_in_odom(self):
        """Newest scan whose odom pose is known: (points (N, 3) in odom, LiDAR position in odom, stamp)."""
        for scan in reversed(self.scans):
            if self.last_scan_stamp is not None and M.to_seconds(scan.header.stamp) <= self.last_scan_stamp:
                return None
            try:
                tf = self.buffer.lookup_transform(self.odom.header.frame_id, scan.header.frame_id,
                                                  Time.from_msg(scan.header.stamp))
            except Exception:
                continue  # odometry for this stamp not in yet; try an older scan
            m = M.transform_matrix(tf.transform)
            self.last_scan_stamp = M.to_seconds(scan.header.stamp)
            return M.cloud_xyz(scan)@m[:3, :3].T+m[:3, 3], m[:3, 3].copy(), self.last_scan_stamp
        return None

    def tick(self):
        if self.job is not None:
            if not self.job.done():
                return
            job, self.job = self.job, None
            try:
                self.finish(*job.result())
            except Exception as exc:
                self.get_logger().error(f"match failed: {type(exc).__name__}: {exc}")
        if self.odom is None:
            return
        if self.pending is not None:
            self.loc.initialize(self.pending, odom_pose(self.odom))
            self.pending = None
            self.set_state("initializing", "searching around the initial pose")
        if self.loc.T is None or self.now()-self.last_match < self.period:
            return
        scan = self.scan_in_odom()
        if scan is None:
            return
        self.last_match = self.now()
        self.job = self.pool.submit(self.match, *scan)

    def match(self, xyz, sensor, stamp):
        begin = time.monotonic()
        return self.loc.update(xyz, sensor), stamp, time.monotonic()-begin

    def finish(self, u, stamp, latency):
        m = u.match
        info = {"message": u.message, "points": u.points, "latency_s": round(latency, 3), "scan_stamp": round(stamp, 3)}
        if m is not None:
            info.update(score=round(m.score, 3), inliers=round(m.inliers, 3), ambiguity=round(m.ambiguity, 3))
        if u.state == "localized":
            self.cov = m.cov
            info.update(correction_m=round(u.correction_m, 3), correction_deg=round(u.correction_deg, 2))
            if u.initializing or self.state != "localized":
                x, y, yaw = self.loc.T
                self.get_logger().info(f"localized: map <- odom = ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg), "
                                       f"{m.inliers:.0%} of {u.points} scan points on map obstacles, "
                                       f"moved {u.correction_m:.2f} m / {u.correction_deg:.1f} deg from the guess")
            self.set_state("localized", **info)
        elif self.loc.initializing:
            self.get_logger().info(f"no fix yet: {u.message}", throttle_duration_sec=5.0)
            self.set_state("initializing", **info)
        else:
            self.set_state("lost" if self.loc.failures >= self.loc.lost_after else "localized", **info)

    def set_state(self, state, message="", **info):
        if state != self.state and message:
            (self.get_logger().warn if state == "lost" else self.get_logger().info)(f"{state}: {message}")
        self.state = state
        T = self.loc.T
        self.pub_status.publish(String(data=json.dumps({
            "state": state, "message": message, "sim_time_s": round(self.now(), 3),
            "map_from_odom": [round(T[0], 3), round(T[1], 3), round(T[2], 4), round(self.loc.z, 3)] if T else None,
            **info})))

    # ------------------------------------------------------------------ outputs
    def publish_tf(self):
        if not self.tf_enabled or not self.loc.localized or self.odom is None:
            return
        x, y, yaw = self.loc.T
        tf = TransformStamped()
        tf.header.stamp = M.stamp(self.now()+self.tolerance)
        tf.header.frame_id, tf.child_frame_id = self.map_frame, self.odom.header.frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, self.loc.z
        tf.transform.rotation = M.yaw_quaternion(yaw)
        self.broadcaster.sendTransform(tf)

    def publish_pose(self):
        if not self.loc.localized or self.odom is None:
            return
        x, y, yaw = compose(self.loc.T, odom_pose(self.odom))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = self.odom.header.stamp, self.map_frame
        msg.pose.pose = M.pose(x, y, yaw)
        msg.pose.pose.position.z = self.odom.pose.pose.position.z+self.loc.z
        cov = np.zeros((6, 6))
        cov[np.ix_([0, 1, 5], [0, 1, 5])] = self.cov
        msg.pose.covariance = [float(v) for v in cov.ravel()]
        self.pub_pose.publish(msg)


def main():
    rclpy.init()
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:  # rclpy can fail to take a message while the launch shuts the graph down
        if rclpy.ok():
            raise
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
