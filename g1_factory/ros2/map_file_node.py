"""map_file: publishes a saved occupancy map (ROS map_server YAML + PGM) as a latched OccupancyGrid.

    python3 -m g1_factory.ros2.map_file_node --ros-args -p yaml:=maps/campus/map_world.yaml -p topic:=/slam_map

Used to run the VLN on the robot's SLAM map instead of the simulator's map (g1_hvln.launch.py map_file:=...).
"""
from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import LATCHED
from g1_factory.slam_map import read_map_yaml


class MapFileNode(Node):
    def __init__(self):
        super().__init__("map_file")
        p = self.declare_parameter
        path, topic, frame = p("yaml", "").value, p("topic", "/slam_map").value, p("frame_id", "map").value
        grid, origin, resolution = read_map_yaml(path)
        pub = self.create_publisher(OccupancyGrid, topic, LATCHED)
        pub.publish(M.occupancy_msg(self.get_clock().now().nanoseconds*1e-9, frame, origin, resolution, grid))
        self.get_logger().info(f"{path}: {grid.shape[1]}x{grid.shape[0]} @ {resolution} m on {topic} ({frame})")


def main():
    rclpy.init()
    node = MapFileNode()
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
