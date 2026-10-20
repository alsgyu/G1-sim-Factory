"""map_file: publishes a saved map as latched topics: the occupancy grid, and with a map package its semantic map.

    python3 -m g1_factory.ros2.map_file_node --ros-args -p yaml:=maps/campus/map_world.yaml -p topic:=/slam_map
    python3 -m g1_factory.ros2.map_file_node --ros-args -p package:=maps/campus -p frame:=map

With `package` it publishes map.yaml on `topic` and semantic_map.json (rooms, places, doorways, anchors) on
`semantic_topic`; frame:=world moves both into the simulator frame (simulation). Used to run the VLN on the robot's
SLAM map (g1_hvln.launch.py map_package:=... or map_file:=...) and to share the map with external agents.
"""
from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import LATCHED
from g1_factory.map_package import load_package
from g1_factory.slam_map import read_map_yaml


class MapFileNode(Node):
    def __init__(self):
        super().__init__("map_file")
        p = self.declare_parameter
        path, topic, frame = p("yaml", "").value, p("topic", "/slam_map").value, p("frame_id", "map").value
        package = p("package", "").value
        now = self.get_clock().now().nanoseconds*1e-9
        if package:
            grid, origin, resolution, semantic = load_package(package, p("frame", "map").value)
            semantic_topic = p("semantic_topic", "/slam/semantic_map").value
            self.pub_semantic = self.create_publisher(String, semantic_topic, LATCHED)
            self.pub_semantic.publish(String(data=json.dumps(semantic, ensure_ascii=False)))
            path = f"{package} ({len(semantic['zones'])} rooms, {len(semantic['edges'])} doorways on {semantic_topic})"
        else:
            grid, origin, resolution = read_map_yaml(path)
        self.pub = self.create_publisher(OccupancyGrid, topic, LATCHED)
        self.pub.publish(M.occupancy_msg(now, frame, origin, resolution, grid))
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
