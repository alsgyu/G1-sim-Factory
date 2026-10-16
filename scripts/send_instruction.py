#!/usr/bin/env python3
"""Minimal external-agent client: send one instruction, stream feedback, save the evidence image.

    python3 scripts/send_instruction.py "검사 구역의 2번 검사대에 가서 부품이 있는지 확인해"
    python3 scripts/send_instruction.py '{"task_id": "a1", "subgoals": ["charging_dock_3"], "target": "charging dock", "question": "Is the dock free?"}'
    python3 scripts/send_instruction.py --cancel

Run in a sourced ROS 2 environment on the same ROS_DOMAIN_ID as the robot/simulator.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

COMMAND = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
EVENTS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE)
EVIDENCE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class Client(Node):
    def __init__(self, payload: str, image_path: Path):
        super().__init__("external_agent_client")
        self.payload, self.image_path, self.sent, self.done = payload, image_path, False, None
        self.pub = self.create_publisher(String, "/vln/instruction", COMMAND)
        self.create_subscription(String, "/vln/feedback", self.on_feedback, EVENTS)
        self.create_subscription(CompressedImage, "/vln/inspection_image", self.on_image, EVIDENCE)
        self.create_timer(0.2, self.tick)

    def tick(self):
        if not self.sent and self.pub.get_subscription_count() > 0:
            self.pub.publish(String(data=self.payload))
            self.sent = True

    def on_feedback(self, msg):
        event = json.loads(msg.data)
        print(json.dumps(event, ensure_ascii=False), flush=True)
        if event.get("state") in ("done", "failed", "canceled"):
            self.done = event["state"]

    def on_image(self, msg):
        self.image_path.write_bytes(bytes(msg.data))
        print(f"evidence image saved: {self.image_path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("instruction", nargs="?", help="Natural language or JSON payload")
    parser.add_argument("--cancel", action="store_true")
    parser.add_argument("--image", type=Path, default=Path("inspection.jpg"))
    parser.add_argument("--timeout", type=float, default=1800.0, help="wall-clock seconds")
    args = parser.parse_args()
    if not args.cancel and not args.instruction:
        parser.error("instruction or --cancel required")
    payload = json.dumps({"cancel": True}) if args.cancel else args.instruction
    if args.image.exists():
        args.image.unlink()  # never report a previous run's evidence
    rclpy.init()
    node = Client(payload, args.image)
    import time
    deadline = time.monotonic()+args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.cancel and node.sent:
                break
            if node.done:
                end = time.monotonic()+3.0  # the evidence image may arrive just after "done"
                while time.monotonic() < end and not node.image_path.exists():
                    rclpy.spin_once(node, timeout_sec=0.2)
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if node.done == "done" or args.cancel else 1)


if __name__ == "__main__":
    main()
