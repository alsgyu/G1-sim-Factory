"""minimap: naive-pp's live minimap as a web page (naive_pp.minimap), fed from ROS 2.

    python3 -m naive_pp.ros.minimap_node --ros-args -p package:=/pp/maps/campus_dev -p port:=8765
    # then open http://localhost:8765 (the Docker container uses the host network)

Shows the package's map with room names, the robot where the localizer puts it (/odom + TF map -> odom, the pose
naive_pp plans from, never the simulator's), the plan, the target room and naive_pp's feedback.

Subscribes
  /odom + /tf            the robot in the map frame (g1_factory.ros2.frames.MapPose)
  /localization/status   std_msgs/String   localizer state
  /vln/instruction       std_msgs/String   instructions as sent
  /vln/feedback          std_msgs/String   naive_pp's events: target, route, goal, arrival
  /plan                  nav_msgs/Path     the path the follower walks
"""
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from naive_pp.minimap import Minimap, serve
from naive_pp.package import load_package
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import TOPICS

HERE = Path(__file__).resolve().parents[2]


class MinimapNode(Node):
    def __init__(self):
        super().__init__("minimap")
        p = lambda name, default: self.declare_parameter(name, default).value
        package = p("package", str(HERE/"maps/campus_dev"))
        host, port = p("host", "0.0.0.0"), int(p("port", 8765))
        self.view = Minimap(load_package(package))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pose = MapPose(self, self.buffer, p("map_frame", "map"))
        sub = lambda name, msg, cb, topic=None: self.create_subscription(msg, topic or name, cb, TOPICS[name][1])
        sub("/odom", Odometry, self.on_odom)
        sub("/localization/status", String, self.on_localization)
        sub("/vln/instruction", String, self.on_instruction, p("instruction_topic", "/vln/instruction"))
        sub("/vln/feedback", String, self.on_feedback, p("feedback_topic", "/vln/feedback"))
        sub("/plan", PathMsg, self.on_plan)
        self.server = serve(self.view, host, port)
        self.get_logger().info(f"minimap of {Path(package).name}: http://localhost:{port} "
                               f"(listening on {host}:{port})")

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self, msg):
        pose = self.map_pose(msg)
        if pose is not None:
            self.view.set_pose(pose, self.now())

    def on_localization(self, msg):
        try:
            self.view.set_localization(json.loads(msg.data).get("state"))
        except (ValueError, AttributeError):
            pass

    def on_instruction(self, msg):
        self.view.on_instruction(msg.data, self.now())

    def on_feedback(self, msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(event, dict):
            self.view.on_feedback(event)

    def on_plan(self, msg):
        points, _ = M.path_to_numpy(msg)
        self.view.set_plan(points)


def main():
    rclpy.init()
    node = MinimapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
