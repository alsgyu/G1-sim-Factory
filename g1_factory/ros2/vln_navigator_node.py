"""/vln_navigator: layer 2 of the hierarchical VLN - vision + instruction -> waypoints.

Receives a step plan from /vln_agent on /vln/nav_request, then loops:
camera image (+ numbered reachable waypoints drawn on it) + instruction + annotated
map context -> VLM decision -> straight segment to the chosen waypoint (or a turn,
or an approach to an object seen in the image) on /plan -> /path_follower.
The SLAM map (/map) only supplies walkable space and annotated region names; the
target's position is never looked up. LiDAR returns update a local obstacle layer;
if one blocks the current segment the node detours locally around it.
"""
from __future__ import annotations

from array import array
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_factory.grounding import SemanticMap
from g1_factory.map_package import load_package, vln_inputs
from g1_factory.planning import CostMap, OccupancyGrid as Grid, PlanningError, astar, path_blocked, smooth_path, densify
from g1_factory.roadmap import RoadMap, region_at
from g1_factory.scene import load_config
from g1_factory.vlm import VLMClient, encode_jpeg
from g1_factory.vln_navigator import (NavStep, NavTask, VLNNavigator, already_in_front, approach_pose,
                                      behind_partition, outside_region)
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]


class VLNNavigatorNode(Node):
    def __init__(self):
        super().__init__("vln_navigator")
        p = self.declare_parameter
        self.client = VLMClient(p("vlm_url", "http://127.0.0.1:11434/v1").value,
                                p("vlm_model", "qwen3-vl:8b-instruct").value,
                                timeout=float(p("vlm_timeout_s", 180.0).value))
        self.standoff = float(p("standoff_m", 1.4).value)
        self.max_decisions = int(p("max_decisions", 40).value)
        self.settle = float(p("settle_s", 0.8).value)
        config = load_config(ROOT/"configs/factory.yaml")
        annotations = SemanticMap.load(config, p("semantic_map", str(ROOT/"configs/semantic_map.yaml")).value).annotated()
        self.regions = {z["id"]: z["bounds"] for z in config["zones"]}
        self.region_labels = {z["id"]: z["label"] for z in config["zones"]} | {"aisle": "aisle / corridor"}
        self.entrances = {z["id"]: tuple(z["goal"]) for z in config["zones"]}
        self.anchors = {pl.label: pl.pose[:2] for pl in annotations.places.values() if pl.kind == "landmark"}
        package = p("map_package", "").value
        self.package = None
        if package:  # rooms, anchors and topology from the map package (a person's annotations of the SLAM map)
            self.package = load_package(package, p("package_frame", "world").value)[3]
            self.regions, self.region_labels, self.entrances, self.anchors, _ = vln_inputs(self.package)
            self.get_logger().info(f"map package {package}: {len(self.regions)} rooms, "
                                   f"{len(self.package['places'])} places, {len(self.package['edges'])} doorways")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pose = MapPose(self, self.buffer)  # /odom in the map frame (map -> odom: simulator or localizer)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        sub("/vln/nav_request", String, self.on_request)
        sub("/map", OccupancyGrid, self.on_map)
        sub("/odom", Odometry, self.on_odom)
        sub("/camera/camera/color/image_raw", Image, self.on_rgb)
        sub("/camera/camera/aligned_depth_to_color/image_raw", Image, self.on_depth)
        sub("/camera/camera/color/camera_info", CameraInfo, self.on_info)
        sub("/livox/lidar", PointCloud2, self.on_cloud)
        sub("/nav/status", String, self.on_nav_status)
        sub("/semantic_map", String, self.on_semantic_map)
        exported = ROOT/"outputs/isaac/semantic_map.json"  # fallback when no node publishes /semantic_map
        self.topology = self.package or (json.loads(exported.read_text()) if exported.is_file() else None)
        pub = lambda name, msg: self.create_publisher(msg, name, TOPICS[name][1])
        self.pub_plan = pub("/plan", PathMsg)
        self.pub_status = pub("/vln/nav_status", String)
        self.pub_image = pub("/vln/decision_image", CompressedImage)
        self.pool = ThreadPoolExecutor(1)
        self.costmap = self.roadmap = self.navigator = None
        self.pose = self.rgb = self.info = None
        self.depths = deque(maxlen=8)
        self.reset()
        self.create_timer(0.2, self.step)
        self.create_timer(0.5, self.watch_path)

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def reset(self):
        self.state, self.request_id, self.job, self.plan_id, self.plan_points = "idle", None, None, None, None
        self.nav_events, self.arrival, self.phase, self.target_xy, self.frame = [], 0.0, "", None, None
        self.scan_views = []

    def on_map(self, msg):
        grid = Grid((msg.info.origin.position.x, msg.info.origin.position.y), msg.info.resolution,
                    M.occupancy_to_numpy(msg))
        self.costmap = CostMap(grid)
        self.roadmap = RoadMap(self.costmap, self.regions)
        self.navigator = VLNNavigator(self.client, self.roadmap, self.region_labels, self.entrances, self.anchors,
                                      topology=self.topology, max_decisions=self.max_decisions)
        self.get_logger().info(f"SLAM map {grid.width}x{grid.height}; {len(self.roadmap.nodes)} waypoints")

    def on_semantic_map(self, msg):
        """Topological layer (places, openings) for the VLN prompt; zones/anchors stay from the annotations."""
        if self.package is not None:  # the map package already brought its own topology
            return
        self.topology = json.loads(msg.data)
        if self.navigator:
            self.navigator.topology = self.topology
        self.get_logger().info(f"semantic map: {len(self.topology.get('places', []))} places, "
                               f"{len(self.topology.get('edges', []))} openings")

    def on_odom(self, msg):
        pose = self.map_pose(msg)
        if pose is None:
            return
        self.pose = pose
        if self.navigator and self.state != "idle":
            self.navigator.note_position(np.array(self.pose[:2]))

    def on_rgb(self, msg):
        self.rgb = msg

    def on_depth(self, msg):
        self.depths.append(msg)

    def on_info(self, msg):
        self.info = msg

    def on_cloud(self, msg):
        if self.costmap is None:
            return
        try:
            tf = self.buffer.lookup_transform("map", msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except Exception:
            return
        xyz = M.cloud_xyz(msg)
        rng = np.linalg.norm(xyz, axis=1)
        xyz = xyz[(rng > 0.3) & (rng < 8.0)]
        matrix = M.transform_matrix(tf.transform)
        world = xyz@matrix[:3, :3].T+matrix[:3, 3]
        world = world[(world[:, 2] > 0.15) & (world[:, 2] < 1.7)]
        self.costmap.mark_points(world[:, :2], self.now())

    def on_nav_status(self, msg):
        status = json.loads(msg.data)
        if status.get("goal_id") == self.plan_id and status.get("state") in ("reached", "failed"):
            self.nav_events.append(status)

    def on_request(self, msg):
        data = json.loads(msg.data)
        if data.get("cancel"):
            request_id = self.request_id
            self.reset()
            self.hold()
            self.report("canceled", request_id=request_id)
            return
        if self.navigator is None:
            self.report("failed", request_id=data.get("request_id"), message="no SLAM map received yet")
            return
        t = data["task"]
        task = NavTask(t["instruction"], [NavStep(s["goal"], s.get("region")) for s in t["steps"]],
                       t.get("target", ""), t.get("question", ""), t.get("language", ""), t.get("target_label", ""))
        self.reset()
        self.hold()  # a new request preempts the current segment
        self.request_id = data.get("request_id")
        self.navigator.start(task)
        self.state, self.arrival = "settling", self.now()
        self.report("started", steps=[s.goal for s in task.steps])

    # ------------------------------------------------------------------ helpers
    def report(self, state, **fields):
        self.pub_status.publish(String(data=json.dumps({"request_id": fields.pop("request_id", self.request_id),
                                                        "state": state, "sim_time_s": round(self.now(), 3),
                                                        **fields}, ensure_ascii=False)))

    def fresh_frame(self):
        """Newest RGB taken after the robot settled, its depth twin and camera pose."""
        if self.rgb is None or self.info is None:
            return None
        stamp = M.to_seconds(self.rgb.header.stamp)
        if stamp < self.arrival+self.settle*0.5:
            return None
        depth = min(self.depths, key=lambda d: abs(M.to_seconds(d.header.stamp)-stamp), default=None)
        if depth is not None and abs(M.to_seconds(depth.header.stamp)-stamp) > 0.1:
            depth = None
        try:
            tf = self.buffer.lookup_transform("map", self.rgb.header.frame_id, Time.from_msg(self.rgb.header.stamp))
        except Exception:
            try:
                tf = self.buffer.lookup_transform("map", self.rgb.header.frame_id, Time())
            except Exception:
                return None
        return self.rgb, depth, M.transform_matrix(tf.transform)

    def hold(self):
        """Stop where the robot stands: a one-point path the follower holds until the next plan."""
        if self.pose is not None:
            self.pub_plan.publish(M.path_msg(self.now(), "map", np.array([self.pose[:2]]), self.pose[2]))

    def send_path(self, points, final_yaw, phase):
        msg = M.path_msg(self.now(), "map", points, final_yaw)
        self.plan_id = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        self.plan_stamp = msg.header.stamp
        for pose in msg.poses:
            pose.header.stamp = msg.header.stamp
        self.plan_points, self.plan_yaw = np.asarray(points), final_yaw
        self.nav_events, self.phase, self.state = [], phase, "executing"
        self.pub_plan.publish(msg)

    def segment(self, goal):
        """Straight segment to a VLN-chosen point; local detour only if an obstacle blocks it."""
        start = np.array(self.pose[:2])
        end = np.array(goal[:2])
        straight = densify(np.array([start, end]), 0.1)
        self.costmap.update(self.now(), start)
        if not path_blocked(self.costmap, straight, 1):
            return straight
        raw, _ = astar(self.costmap, start, end, max_expansions=60000)  # local avoidance, bounded
        return densify(smooth_path(self.costmap, raw), 0.1)

    def watch_path(self):
        """Low-level safety: detour around obstacles the LiDAR sees on the current segment."""
        if self.state != "executing" or self.plan_points is None or self.pose is None or self.phase in ("turn", "scan"):
            return
        xy = np.array(self.pose[:2])
        if np.linalg.norm(self.plan_points[-1]-xy) < 0.5:
            return
        self.costmap.update(self.now(), xy)
        progress = int(np.argmin(np.linalg.norm(self.plan_points-xy, axis=1)))
        if path_blocked(self.costmap, self.plan_points, progress):
            try:
                points = self.segment((*self.plan_points[-1], self.plan_yaw))
            except PlanningError as exc:
                self.report("blocked", message=str(exc))
                self.hold()
                self.state, self.arrival = "settling", self.now()  # let the VLN choose another way
                self.navigator.history.append("the chosen waypoint was blocked by an obstacle")
                return
            msg = M.path_msg(self.now(), "map", points, self.plan_yaw)
            msg.header.stamp = self.plan_stamp  # same plan id: the follower treats it as a replan
            for pose in msg.poses:
                pose.header.stamp = self.plan_stamp
            self.plan_points = np.asarray(points)
            self.pub_plan.publish(msg)
            self.report("detour", poses=len(points))

    # ------------------------------------------------------------------ loop
    def step(self):
        if self.job is not None:
            if not self.job.done():
                return
            job, self.job = self.job, None
            try:
                self.act(job.result())
            except Exception as exc:
                self.report("failed", message=f"{type(exc).__name__}: {exc}")
                self.state = "idle"
            return
        if self.state == "executing" and self.nav_events:
            event = self.nav_events.pop(0)
            if self.phase == "scan":  # one look-around turn done (or as far as it got): take the next view
                self.state, self.arrival = "scan_settling", float(event.get("sim_time_s", self.now()))
                return
            if event["state"] == "failed":
                self.navigator.history.append("could not reach the chosen waypoint")
            elif self.phase == "approach":
                # A confirmed target was approached to its stand-off pose: the step is done.
                self.navigator.history.append("standing in front of the target")
                self.finish_step(float(event.get("sim_time_s", self.now())))
                return
            self.state, self.arrival = "settling", float(event.get("sim_time_s", self.now()))
        elif self.state == "settling" and self.now()-self.arrival >= self.settle and self.pose is not None:
            xy = np.array(self.pose[:2])
            while self.navigator.auto_advance(xy):
                self.report("step_done", step=self.navigator.task.index, auto=True)
            frame = self.fresh_frame()
            if frame is None:
                return
            rgb_msg, depth_msg, map_from_cam = frame
            rgb = M.image_to_numpy(rgb_msg)
            k = self.info.k
            intrinsics = (k[0], k[4], k[2], k[5], self.info.width, self.info.height)
            pose = self.pose
            if self.navigator.needs_scan(xy):
                self.scan_views = [(rgb, pose[2])]
                self.scan_turn()
                return
            self.frame = (rgb_msg, depth_msg, map_from_cam)
            self.state = "deciding"
            self.job = self.pool.submit(self.navigator.decide, np.array(pose[:2]), pose[2], rgb, map_from_cam,
                                        intrinsics)
        elif self.state == "scan_settling" and self.now()-self.arrival >= self.settle and self.pose is not None:
            frame = self.fresh_frame()
            if frame is None:
                return
            self.scan_views.append((M.image_to_numpy(frame[0]), self.pose[2]))
            if len(self.scan_views) < 4:
                self.scan_turn()
                return
            self.state = "deciding"
            self.job = self.pool.submit(self.navigator.choose_view, np.array(self.pose[:2]), list(self.scan_views))

    def scan_turn(self):
        """Next 90-degree look-around turn, relative to the first view (no drift accumulation)."""
        heading = self.scan_views[0][1]+len(self.scan_views)*math.pi/2
        self.send_path(np.array([self.pose[:2]]), math.atan2(math.sin(heading), math.cos(heading)), "scan")

    def act(self, d):
        if d.prompt_image is not None:
            image = CompressedImage()
            image.header.stamp, image.header.frame_id = M.stamp(self.now()), "camera_color_optical_frame"
            image.format = "jpeg"
            image.data = array("B", encode_jpeg(d.prompt_image, 80))
            self.pub_image.publish(image)
        info = {"action": d.action, "thought": d.thought, "vlm_latency_s": round(d.latency_s, 2),
                "request_bytes": d.request_bytes, "step": self.navigator.task.index if self.navigator.task else None,
                "note": d.note, "raw": d.raw[:400],
                "pose": [round(float(v), 2) for v in self.pose] if self.pose is not None else None}
        if d.plate is not None:
            info["plate"] = d.plate
        if d.waypoint is not None:
            info.update(waypoint=d.waypoint.id, waypoint_xy=[round(float(v), 2) for v in d.waypoint.xy],
                        waypoint_region=d.waypoint.region)
        self.report("decision", **info)
        if d.action in ("move", "turn"):
            if d.action == "turn":
                self.send_path(np.array([self.pose[:2]]), d.goal[2], "turn")
            else:
                try:
                    self.send_path(self.segment(d.goal), d.goal[2], "move")
                except PlanningError as exc:
                    self.navigator.history.append(f"waypoint unreachable ({exc})")
                    self.state, self.arrival = "settling", self.now()
            return
        if d.action == "approach":
            located = self.localize(d.bbox)
            if located is None:
                self.navigator.history.append("the target box had no valid depth; look again")
                self.state, self.arrival = "settling", self.now()
                return
            center, near = located
            if behind_partition(center[:2], self.pose[:2], self.costmap.occupied):
                self.navigator.history.append("the object you selected is behind a wall or glass partition (depth "
                                              "hit the wall): walk around to it through a doorway first")
                self.state, self.arrival = "settling", self.now()
                return
            step = self.navigator.task.current if self.navigator.task else None
            if step and outside_region(center[:2], self.roadmap.regions, step.region):
                found = self.region_labels.get(region_at(self.roadmap.regions, center[:2]), "another area")
                self.navigator.history.append(
                    f"the object you selected is in {found}, not in {self.region_labels.get(step.region, step.region)}: "
                    "look for the target inside that area")
                self.state, self.arrival = "settling", self.now()
                return
            self.target_xy = [round(float(center[0]), 3), round(float(center[1]), 3)]
            goal = approach_pose(center[:2], near[:2], self.pose[:2], self.standoff, self.costmap.occupied)
            self.report("target_located", target_xy=self.target_xy, approach=[round(v, 3) for v in goal])
            snapped = self.costmap.nearest_traversable(goal[:2], 1.5)
            if snapped is not None and already_in_front(snapped, self.pose[:2]):
                self.navigator.history.append("standing in front of the target")
                d.action = "stop"
                return self.act(d)
            if snapped is None:
                self.navigator.history.append("no free floor in front of the target")
                self.state, self.arrival = "settling", self.now()
                return
            try:
                self.send_path(self.segment((snapped[0], snapped[1], goal[2])), goal[2], "approach")
            except PlanningError as exc:
                self.navigator.history.append(f"cannot reach the target ({exc})")
                self.state, self.arrival = "settling", self.now()
            return
        if d.action == "stop":
            self.finish_step(self.now())
            return
        if d.action == "finished":
            self.state = "idle"
            self.report("arrived", final_pose=[round(v, 3) for v in self.pose], target_xy=self.target_xy,
                        decisions=self.navigator.decisions)
            return
        self.state = "idle"
        self.report("failed", message=d.note or d.action, decisions=self.navigator.decisions)

    def finish_step(self, t):
        finished = self.navigator.complete_step()
        self.report("step_done", step=self.navigator.task.index)
        if finished:
            self.state = "idle"
            self.report("arrived", final_pose=[round(v, 3) for v in self.pose], target_xy=self.target_xy,
                        decisions=self.navigator.decisions)
        else:
            self.state, self.arrival, self.phase = "settling", t, ""

    def localize(self, box):
        rgb_msg, depth_msg, map_from_cam = self.frame
        if depth_msg is None:
            return None
        depth = M.depth_to_metres(depth_msg)
        x1, y1, x2, y2 = box
        cx, cy, w, h = (x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1
        patch = depth[int(max(0, cy-0.2*h)):int(min(depth.shape[0], cy+0.2*h+1)),
                      int(max(0, cx-0.2*w)):int(min(depth.shape[1], cx+0.2*w+1))]
        valid = patch[patch > 0.1]
        if valid.size < 5:
            return None
        k = self.info.k
        out = []
        for z in (float(np.median(valid)), float(np.percentile(valid, 10))):
            out.append((map_from_cam@np.array([(cx-k[2])*z/k[0], (cy-k[5])*z/k[4], z, 1.0]))[:3])
        return out


def main():
    rclpy.init()
    node = VLNNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
