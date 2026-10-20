"""Robot pose in the map frame: /odom composed with map -> odom from TF.

The VLN nodes plan, follow and report in the map frame. With the simulator's map (package_frame:=world) map -> odom
is the simulator's static identity; on a robot localised in a map package (package_frame:=map) the localizer node
publishes it. Reading /odom as if odom were the map is only right in the first case, so nodes go through MapPose.
"""
from __future__ import annotations

import math

from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from g1_factory.ros2 import messages as M


class MapPose:
    """(x, y, yaw) in the map frame of a nav_msgs/Odometry, with the latest map -> odom; None before it is known."""

    def __init__(self, node, buffer: Buffer | None = None, frame: str = "map"):
        self.frame = frame
        if buffer is None:
            buffer = Buffer()
            self.listener = TransformListener(buffer, node)
        self.buffer = buffer

    def __call__(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:  # latest map -> odom: it changes slowly, and odom carries the motion since
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id, Time())
        except Exception:
            return None  # not localised yet: the caller waits, as without odometry
        m = M.transform_matrix(tf.transform)
        xyz = m[:3, :3]@[p.x, p.y, p.z]+m[:3, 3]
        heading = m[:3, :3]@M.quaternion_matrix(q)[:, 0]
        return float(xyz[0]), float(xyz[1]), math.atan2(heading[1], heading[0])
