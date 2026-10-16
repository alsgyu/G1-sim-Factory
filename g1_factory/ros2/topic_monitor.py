"""/topic_monitor: measures every topic of the stack from the live ROS 2 graph.

* serialized message size (CDR bytes as carried by DDS, raw=True subscription)
* message count and rate in simulation time and wall time
* actual publisher/subscriber endpoints and their QoS (rclpy graph API)
Writes <output_dir>/topic_stats.json every 10 s and on shutdown. The monitor's
own subscriptions are excluded from the reported endpoints.
"""
from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from g1_factory.ros2.qos import TOPICS, describe


def qos_dict(qos):
    return {"reliability": qos.reliability.name, "durability": qos.durability.name,
            "history": qos.history.name, "depth": qos.depth}


class TopicMonitor(Node):
    def __init__(self):
        super().__init__("topic_monitor")
        self.out = Path(self.declare_parameter("output_dir", "outputs/ros2_run").value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.stats = {}
        for name, (type_name, qos) in TOPICS.items():
            self.stats[name] = {"type": type_name, "count": 0, "bytes": 0, "min_bytes": None, "max_bytes": 0,
                                "first_sim": None, "last_sim": None, "first_wall": None, "last_wall": None,
                                "declared_qos": describe(qos)}
            self.create_subscription(get_message(type_name), name, self._callback(name), qos, raw=True)
        self.endpoints = {}
        self.create_timer(10.0, self.snapshot)

    def _callback(self, name):
        stat = self.stats[name]

        def on_message(raw: bytes):
            size = len(raw)
            sim = self.get_clock().now().nanoseconds*1e-9
            wall = time.monotonic()
            stat["count"] += 1
            stat["bytes"] += size
            stat["min_bytes"] = size if stat["min_bytes"] is None else min(stat["min_bytes"], size)
            stat["max_bytes"] = max(stat["max_bytes"], size)
            if stat["first_sim"] is None:
                stat["first_sim"], stat["first_wall"] = sim, wall
            stat["last_sim"], stat["last_wall"] = sim, wall
        return on_message

    def graph(self):
        for name in TOPICS:
            pubs = [{"node": i.node_name, "qos": qos_dict(i.qos_profile)}
                    for i in self.get_publishers_info_by_topic(name)]
            subs = [{"node": i.node_name, "qos": qos_dict(i.qos_profile)}
                    for i in self.get_subscriptions_info_by_topic(name) if i.node_name != self.get_name()]
            if pubs or subs:
                # Keep the richest observation: nodes may still be starting at the first snapshot.
                old = self.endpoints.get(name, {"publishers": [], "subscribers": []})
                self.endpoints[name] = {"publishers": pubs if len(pubs) >= len(old["publishers"]) else old["publishers"],
                                        "subscribers": subs if len(subs) >= len(old["subscribers"]) else old["subscribers"]}

    def snapshot(self):
        try:
            self.graph()
        except Exception:  # graph queries fail once the context is shutting down
            pass
        report = {}
        for name, s in self.stats.items():
            entry = dict(s)
            n = s["count"]
            if n:
                entry["mean_bytes"] = s["bytes"]/n
            if n > 1 and s["last_sim"] > s["first_sim"]:
                entry["rate_hz_sim"] = (n-1)/(s["last_sim"]-s["first_sim"])
            if n > 1 and s["last_wall"] > s["first_wall"]:
                entry["rate_hz_wall"] = (n-1)/(s["last_wall"]-s["first_wall"])
                entry["bandwidth_Bps_wall"] = s["bytes"]/(s["last_wall"]-s["first_wall"])
            entry.update(self.endpoints.get(name, {}))
            report[name] = entry
        (self.out/"topic_stats.json").write_text(json.dumps(report, indent=2)+"\n")


def main():
    rclpy.init()
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.snapshot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
