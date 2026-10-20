"""ROS 2 side of a G1 simulator: identical topics, frames and QoS for MuJoCo and Isaac Sim.

Simulator nodes (g1_factory.ros2.sim_node for MuJoCo, g1_factory.isaac.isaac_sim_node
for Isaac Sim) own the physics and sensors and pass plain numpy data here.
"""
from __future__ import annotations

import json
import math
import time

import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS

OPTICAL = (-0.5, 0.5, -0.5, 0.5)  # x,y,z,w: camera_link (x fwd) -> optical (z fwd, x right, y down)
CMD_LIMITS = (np.array([-0.6, -0.3, -1.0]), np.array([0.8, 0.3, 1.0]))  # conservative safety clamp


class SimTopics:
    """Publishers for the G1 head sensors, state, map and clock, plus the /cmd_vel input."""

    def __init__(self, node, now_fn):
        self.node, self.now = node, now_fn
        pub = lambda name, msg: node.create_publisher(msg, name, TOPICS[name][1])
        self.clock = pub("/clock", Clock)
        self.rgb = pub("/camera/camera/color/image_raw", Image)
        self.rgb_info = pub("/camera/camera/color/camera_info", CameraInfo)
        self.depth = pub("/camera/camera/aligned_depth_to_color/image_raw", Image)
        self.depth_info = pub("/camera/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self.cloud = pub("/livox/lidar", PointCloud2)
        self.imu = pub("/livox/imu", Imu)
        self.odom = pub("/odom", Odometry)
        self.joints = pub("/joint_states", JointState)
        self.tf = pub("/tf", TFMessage)
        self.tf_static = pub("/tf_static", TFMessage)
        self.map = pub("/map", OccupancyGrid)
        self.semantic_map = pub("/semantic_map", String)
        self.status = pub("/sim/status", String)
        self.truth = None  # /sim/ground_truth, only when /odom is made to drift (publish_truth)
        node.create_subscription(Twist, "/cmd_vel", self._on_cmd, TOPICS["/cmd_vel"][1])
        self.command = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = -math.inf
        self.timing = {}

    def _on_cmd(self, msg: Twist):
        cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        if not np.isfinite(cmd).all():
            self.node.get_logger().warn("Ignoring non-finite /cmd_vel")
            return
        self.command = np.clip(cmd, *CMD_LIMITS).astype(np.float32)
        self.last_cmd_time = self.now()

    def active_command(self, t, timeout, settle=1.0):
        """Zero while the policy settles after reset, or when commands go stale."""
        if t < settle or t-self.last_cmd_time > timeout:
            return np.zeros(3, dtype=np.float32)
        return self.command

    def timed(self, key, begin):
        total, n = self.timing.get(key, (0.0, 0))
        self.timing[key] = (total+time.perf_counter()-begin, n+1)

    # ------------------------------------------------------------------ static
    def publish_static(self, t, camera_xyz, camera_pitch, lidar_xyz, grid, semantic_json=None, map_tf=True):
        """map_tf=False leaves map -> odom to a localizer (the VLN in the frame of a map package)."""
        def tf(parent, child, xyz, quat):
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = M.stamp(t), parent, child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = map(float, quat)
            return msg
        self.tf_static.publish(TFMessage(transforms=[
            *([tf("map", "odom", (0, 0, 0), (0, 0, 0, 1))] if map_tf else []),
            tf("base_link", "camera_link", camera_xyz, (0, math.sin(camera_pitch/2), 0, math.cos(camera_pitch/2))),
            tf("camera_link", "camera_color_optical_frame", (0, 0, 0), OPTICAL),
            tf("base_link", "livox_frame", lidar_xyz, (0, 0, 0, 1)),
        ]))
        self.map.publish(M.occupancy_msg(t, "map", grid.origin, grid.resolution, grid.data))
        if semantic_json:  # zones, objects, places/edges/anchors (g1_factory.semantic_world)
            self.semantic_map.publish(String(data=semantic_json))

    # ------------------------------------------------------------------ periodic
    def publish_clock(self, t):
        self.clock.publish(Clock(clock=M.stamp(t)))

    def publish_camera(self, t, rgb, depth_m, fx, fy, cx, cy):
        begin = time.perf_counter()
        frame = "camera_color_optical_frame"
        h, w = rgb.shape[:2]
        info = M.camera_info_msg(t, frame, w, h, fx, fy, cx, cy)
        self.rgb.publish(M.image_msg(t, frame, rgb, "rgb8"))
        self.rgb_info.publish(info)
        depth_mm = np.round(np.clip(np.nan_to_num(depth_m, nan=0.0, posinf=0.0), 0, 65.535)*1000).astype(np.uint16)
        self.depth.publish(M.image_msg(t, frame, depth_mm, "16UC1"))
        self.depth_info.publish(info)
        self.timed("camera_publish", begin)

    def publish_lidar(self, t, xyz, intensity, line, point_ns):
        begin = time.perf_counter()
        self.cloud.publish(M.livox_cloud_msg(t, "livox_frame", xyz, intensity, line, point_ns))
        self.timed("lidar_publish", begin)

    def publish_imu(self, t, gyro, accel):
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = M.stamp(t), "livox_frame"
        msg.orientation_covariance[0] = -1.0  # the MID-360 IMU reports no orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)
        self.imu.publish(msg)

    def publish_state(self, t, position, quat_wxyz, lin_vel_body, ang_vel_body, joint_names, q, dq, tau):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = M.stamp(t), "odom", "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x = float(quat_wxyz[0]), float(quat_wxyz[1])
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = float(quat_wxyz[2]), float(quat_wxyz[3])
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, lin_vel_body)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, ang_vel_body)
        self.odom.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, position)
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.publish(TFMessage(transforms=[tf]))
        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = list(joint_names)
        js.position = [float(v) for v in q]
        js.velocity = [float(v) for v in dq]
        js.effort = [float(v) for v in tau]
        self.joints.publish(js)

    def publish_truth(self, t, position, quat_wxyz):
        """True base pose (frame world) for the evaluator when /odom drifts."""
        if self.truth is None:
            self.truth = self.node.create_publisher(Odometry, "/sim/ground_truth", TOPICS["/sim/ground_truth"][1])
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = M.stamp(t), "world", "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, position)
        (msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y, msg.pose.pose.orientation.z) = map(float, quat_wxyz)
        self.truth.publish(msg)

    def publish_status(self, t, **fields):
        fields["timing_ms_mean"] = {k: round(1000*v[0]/max(v[1], 1), 2) for k, v in self.timing.items()}
        fields["command"] = [round(float(v), 3) for v in self.command]
        self.status.publish(String(data=json.dumps({"sim_time_s": round(t, 3), **fields})))


class Scheduler:
    """Fixed-rate publishing on simulation time ('is it due?')."""

    def __init__(self, **rates_hz):
        self.rates = rates_hz
        self.next = {k: 0.0 for k in rates_hz}

    def due(self, key, t):
        if t+1e-9 >= self.next[key]:
            self.next[key] = max(self.next[key]+1.0/self.rates[key], t)
            return True
        return False
