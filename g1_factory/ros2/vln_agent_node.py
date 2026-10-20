"""/vln_agent: task executive and layer 1 of the hierarchical VLN stack.

planner_mode:=vln (default, the VLN):
  external agent --/vln/instruction--> VLM decomposes the instruction into steps
  over annotated regions --/vln/nav_request--> /vln_navigator (layer 2: picks
  waypoints from camera images + instruction) --/plan--> /path_follower -->
  /cmd_vel --> G1 policy. On "arrived" the VLM answers the check question.
planner_mode:=oracle | rule (baselines, not VLN): target place looked up in the
  semantic map (by VLM or alias matching) and sent to /global_planner (A*).
Progress and results go back on /vln/feedback and /vln/inspection_image. VLM
calls run in a worker thread; the node keeps spinning.
"""
from __future__ import annotations

from array import array
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from g1_factory.grounding import GroundingError, SemanticMap, ground_instruction
from g1_factory.map_package import load_package, vln_inputs
from g1_factory.scene import load_config
from g1_factory.vlm import HierarchicalVLM, VLMClient, encode_jpeg
from g1_factory.vln_navigator import NavStep, NavTask, decompose
import uuid
from g1_factory.ros2 import messages as M
from g1_factory.ros2.frames import MapPose
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]


class VLNAgentNode(Node):
    def __init__(self):
        super().__init__("vln_agent")
        p = self.declare_parameter
        mode = p("planner_mode", "vln").value
        self.mode = {"vlm": "oracle"}.get(mode, mode)
        if self.mode not in ("vln", "oracle", "rule"):
            raise ValueError("planner_mode must be vln, oracle or rule")
        config = load_config(ROOT/"configs/factory.yaml")
        self.smap = SemanticMap.load(config, p("semantic_map", str(ROOT/"configs/semantic_map.yaml")).value)
        self.regions = {z["id"]: z["bounds"] for z in config["zones"]}
        self.region_labels = {z["id"]: z["label"] for z in config["zones"]}
        package = p("map_package", "").value
        if package:  # rooms a person annotated on the robot's SLAM map, instead of the simulator's zones
            semantic = load_package(package, p("package_frame", "world").value)[3]
            regions, labels, _, _, aliases = vln_inputs(semantic)
            self.regions = regions
            self.region_labels = {rid: labels[rid]+(f" ({', '.join(aliases[rid])})" if aliases.get(rid) else "")
                                  for rid in regions}
        self.client = VLMClient(p("vlm_url", "http://127.0.0.1:11434/v1").value,
                                p("vlm_model", "qwen3-vl:8b-instruct").value,
                                timeout=float(p("vlm_timeout_s", 180.0).value))
        self.vlm = HierarchicalVLM(self.client, self.smap)
        self.standoff = float(p("inspect_standoff_m", 1.4).value)
        self.look_around_deg = float(p("look_around_deg", 40.0).value)
        self.settle = float(p("settle_s", 1.0).value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pose = MapPose(self, self.buffer)  # /odom in the map frame (map -> odom: simulator or localizer)
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        sub("/vln/instruction", String, self.on_instruction)
        sub("/nav/status", String, self.on_nav_status)
        sub("/vln/nav_status", String, self.on_vln_status)
        sub("/odom", Odometry, self.on_odom)
        sub("/camera/camera/color/image_raw", Image, self.on_rgb)
        sub("/camera/camera/aligned_depth_to_color/image_raw", Image, self.on_depth)
        sub("/camera/camera/color/camera_info", CameraInfo, self.on_info)
        pub = lambda name, msg: self.create_publisher(msg, name, TOPICS[name][1])
        self.pub_feedback = pub("/vln/feedback", String)
        self.pub_evidence = pub("/vln/inspection_image", CompressedImage)
        if self.mode == "vln":
            self.pub_nav_request = pub("/vln/nav_request", String)
        else:  # map-lookup baselines: target poses go to the A* global_planner
            self.pub_goal = pub("/goal_pose", PoseStamped)
            self.pub_subgoals = pub("/vln/subgoals", PoseArray)
        self.pool = ThreadPoolExecutor(1)
        self.rgb = self.info = self.pose = None
        self.depths = deque(maxlen=8)  # pair each RGB frame with the depth frame of the same stamp
        self.reset_task()
        self.create_timer(0.2, self.step)
        self.get_logger().info(f"VLN agent ready ({self.mode}); {len(self.smap.places)} semantic places")

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self, msg):
        self.pose = self.map_pose(msg) or self.pose

    def on_rgb(self, msg):
        self.rgb = msg

    def on_depth(self, msg):
        self.depths.append(msg)

    def on_info(self, msg):
        self.info = msg

    def on_instruction(self, msg: String):
        text = msg.data
        try:
            data = json.loads(text) if text.lstrip().startswith("{") else None
        except json.JSONDecodeError:
            data = None
        if data and data.get("cancel"):
            if self.state not in ("idle", "done", "failed"):
                self.hold_position()
            self.feedback("canceled", message="canceled by external agent")
            self.reset_task()
            return
        if self.state not in ("idle", "done", "failed"):
            self.hold_position()
            self.feedback("preempted", message="new instruction received")
        self.reset_task()
        self.instruction, self.state, self.t_accept = text, "planning", self.now()
        self.task_id = str((data or {}).get("task_id") or uuid.uuid4().hex[:8])
        self.feedback("accepted", instruction=(data or {}).get("instruction", text) if data else text)
        if self.mode == "vln":
            self.job_kind, self.job = "decompose", self.pool.submit(self._decompose, data, text)
        elif self.mode == "rule":
            self.job_kind, self.job = "plan", self.pool.submit(self._rule_plan, text)
        else:
            self.job_kind, self.job = "plan", self.pool.submit(self.vlm.plan_task, text)

    def _decompose(self, data, text):
        """Layer 1 for the VLN: explicit `steps` from the agent, or VLM decomposition."""
        if data and "steps" in data:
            steps = [NavStep(str(s["goal"]), s.get("region")) for s in data["steps"]]
            bad = [s.region for s in steps if s.region is not None and s.region not in self.regions]
            if not steps or bad:
                raise ValueError(f"steps need goals and known regions; unknown: {bad}")
            task = NavTask(str(data.get("instruction", "")) or steps[-1].goal, steps,
                           str(data.get("target", steps[-1].goal)), str(data.get("question", "")),
                           str(data.get("language", "")), str(data.get("target_label", "")))
            return task, {"source": "external_steps"}
        instruction = data.get("instruction", "") if data else text
        if not str(instruction).strip():
            raise ValueError("the VLN needs an `instruction` or `steps`; `subgoals` (map places/coordinates) "
                             "are only followed by planner_mode:=oracle")
        task, meta = decompose(self.client, instruction, self.regions, self.region_labels)
        return task, {"source": "vlm", **meta}

    def on_vln_status(self, msg: String):
        status = json.loads(msg.data)
        if self.mode != "vln" or status.get("request_id") != self.task_id or self.state != "vln_navigating":
            return
        state = status.get("state")
        if state == "decision":
            self.vlm_log.append({"source": "navigate", "latency_s": status.get("vlm_latency_s"),
                                 "request_bytes": status.get("request_bytes"), "action": status.get("action"),
                                 "raw": status.get("raw")})
            self.feedback("decision", **{k: status.get(k) for k in ("action", "thought", "waypoint", "waypoint_xy",
                                                                     "waypoint_region", "step", "plate", "note", "pose",
                                                                     "vlm_latency_s")})
        elif state in ("step_done", "target_located", "detour", "blocked"):
            if status.get("target_xy"):
                self.target_xy = status["target_xy"]
            self.feedback(state, **{k: v for k, v in status.items() if k not in ("request_id", "state", "sim_time_s")})
        elif state == "arrived":
            self.target_xy = status.get("target_xy") or self.target_xy
            self.feedback("arrived", decisions=status.get("decisions"), final_pose=status.get("final_pose"))
            self.state, self.phase, self.arrival = "settling", "inspect", float(status.get("sim_time_s", self.now()))
        elif state == "failed":
            self.fail(f"navigation failed: {status.get('message', '')}")

    def _rule_plan(self, text):
        task = ground_instruction(text, self.smap)
        data = json.loads(text) if text.lstrip().startswith("{") else {}
        return task, {"source": "rule", "target": data.get("target", ""), "question": data.get("question", ""),
                      "language": data.get("language", "")}

    def on_nav_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if status.get("goal_id") == self.goal_id and status.get("state") in ("reached", "failed", "plan_failed"):
            self.nav_events.append(status)
        if status.get("goal_id") == self.goal_id and status.get("state") == "planned":
            self.last_plan = status

    # ----------------------------------------------------------------- state
    def reset_task(self):
        self.state, self.task, self.meta, self.index = "idle", None, {}, 0
        self.goal_id, self.goal, self.nav_events, self.retries = None, None, [], 0
        self.instruction, self.phase, self.views, self.arrival = "", "", [], None
        self.result, self.last_plan, self.vlm_log = None, None, []
        self.t_accept, self.target_xy, self.task_id = 0.0, None, None
        # Drop any running VLM job: its result belongs to the previous task.
        self.job, self.job_kind = None, None

    def feedback(self, state, **fields):
        task_id = self.task_id
        self.pub_feedback.publish(String(data=json.dumps({"task_id": task_id, "state": state,
                                                          "sim_time_s": round(self.now(), 3), **fields},
                                                         ensure_ascii=False)))

    def hold_position(self):
        """Stop and hold the current pose (VLN: the navigator holds; oracle/rule: a goal at the own pose)."""
        if self.mode == "vln":
            self.pub_nav_request.publish(String(data=json.dumps({"cancel": True})))
        elif self.pose is not None:
            self.pub_goal.publish(M.pose_stamped(self.now(), "map", *self.pose))

    def send_goal(self, x, y, yaw, phase):
        self.goal = (x, y, yaw)
        msg = M.pose_stamped(self.now(), "map", x, y, yaw)
        self.goal_id = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        self.nav_events, self.phase, self.state = [], phase, "navigating"
        self.pub_goal.publish(msg)

    def step(self):
        if self.job is not None and self.job.done():
            job, kind = self.job, self.job_kind
            self.job = self.job_kind = None
            try:
                self.on_job(kind, job.result())
            except Exception as exc:  # VLM/server errors are reported, never silently replaced
                self.fail(f"{kind} failed: {type(exc).__name__}: {exc}")
            return
        if self.state == "navigating" and self.nav_events:
            event = self.nav_events.pop(0)
            if event["state"] == "reached":
                self.on_reached(event)
            elif self.retries < 1:
                self.retries += 1
                self.get_logger().warn(f"Navigation {event['state']}: {event.get('message')}; retrying once")
                self.send_goal(*self.goal, self.phase)
            elif self.phase in ("refine", "look", "inspect"):
                # The place itself was reached; a failed visual re-approach is not a task failure.
                self.feedback("refine_skipped", message=f"{event['state']}: {event.get('message', '')}")
                self.phase = "inspect"
                self.request_inspect()
            else:
                self.hold_position()
                self.fail(f"navigation {event['state']}: {event.get('message', '')}")
        elif self.state == "settling" and self.now()-self.arrival >= self.settle:
            self.after_settle()

    def on_job(self, kind, value):
        if kind == "decompose":
            task, meta = value
            self.task, self.meta = task, {"target": task.target, "question": task.question,
                                          "language": task.language, "place": task.target}
            self.vlm_log.append({"source": "decompose", **{k: meta.get(k) for k in
                                 ("latency_s", "request_bytes", "response_bytes", "raw")}})
            self.feedback("planned", parser=meta.get("source"), steps=[{"goal": s.goal, "region": s.region}
                                                                       for s in task.steps],
                          target=task.target, target_label=task.target_label, question=task.question,
                          vlm_latency_s=meta.get("latency_s"))
            self.pub_nav_request.publish(String(data=json.dumps({"request_id": self.task_id, "task": {
                "instruction": task.instruction, "target": task.target, "question": task.question,
                "language": task.language, "target_label": task.target_label,
                "steps": [{"goal": s.goal, "region": s.region} for s in task.steps]}},
                ensure_ascii=False)))
            self.state = "vln_navigating"
        elif kind == "plan":
            task, meta = value
            self.task, self.meta = task, meta
            if not task.ok:
                self.fail("instruction could not be grounded", unresolved=task.unresolved, suggestions=task.suggestions)
                return
            self.vlm_log.append({k: meta.get(k) for k in ("source", "latency_s", "attempts", "request_bytes",
                                                          "response_bytes", "reason", "raw")})
            poses = PoseArray()
            poses.header.stamp, poses.header.frame_id = M.stamp(self.now()), "map"
            poses.poses = [M.pose(s.x, s.y, s.yaw) for s in task.subgoals]
            self.pub_subgoals.publish(poses)
            self.feedback("planned", parser=task.parser, subgoals=[s.place_id for s in task.subgoals],
                          target=meta.get("target"), question=meta.get("question"), reason=meta.get("reason"),
                          vlm_latency_s=meta.get("latency_s"))
            self.index = 0
            self.next_subgoal()
        elif kind == "locate":
            box, call, image_stamp = value
            self.vlm_log.append({"source": "locate", "latency_s": call.latency_s, "request_bytes": call.request_bytes,
                                 "response_bytes": call.response_bytes, "raw": call.text})
            self.on_located(box, image_stamp, call)
        elif kind == "inspect":
            result, call, jpeg = value
            self.vlm_log.append({"source": "inspect", "latency_s": call.latency_s, "request_bytes": call.request_bytes,
                                 "response_bytes": call.response_bytes, "raw": call.text})
            evidence = CompressedImage()
            evidence.header.stamp, evidence.header.frame_id = M.stamp(self.now()), "camera_color_optical_frame"
            evidence.format = "jpeg"
            evidence.data = array("B", jpeg)
            self.pub_evidence.publish(evidence)
            self.state = "done"
            self.feedback("done", result=result, final_pose=[round(v, 3) for v in self.pose] if self.pose else None,
                          target_xy=self.target_xy, elapsed_s=round(self.now()-self.t_accept, 2), vlm_calls=self.vlm_log)

    def next_subgoal(self):
        sg = self.task.subgoals[self.index]
        self.retries = 0
        self.feedback("navigating", subgoal_index=self.index, place=sg.place_id, label=sg.label,
                      goal=[sg.x, sg.y, sg.yaw])
        self.send_goal(sg.x, sg.y, sg.yaw, "subgoal")

    def on_reached(self, event):
        if self.phase == "subgoal":
            sg = self.task.subgoals[self.index]
            self.feedback("subgoal_reached", subgoal_index=self.index, place=sg.place_id,
                          cross_track_rmse_m=event.get("cross_track_rmse_m"), elapsed_s=event.get("elapsed_s"))
            self.index += 1
            if self.index < len(self.task.subgoals):
                self.next_subgoal()
                return
            self.views = [0.0, self.look_around_deg, -self.look_around_deg]
            self.target_xy = None
        # Images must be captured after the robot stopped: use the follower's sim time.
        self.state, self.arrival = "settling", float(event.get("sim_time_s", self.now()))

    def after_settle(self):
        """Arrived somewhere and the camera has settled: decide the next visual step."""
        target = (self.meta or {}).get("target", "")
        if self.phase in ("subgoal", "look") and target and self.target_xy is None:
            self.request_locate(target)
        else:
            self.request_inspect()

    def latest_frames(self):
        """Newest RGB captured after the robot settled, and the depth frame closest in time."""
        if self.rgb is None:
            raise RuntimeError("no camera image received")
        stamp = M.to_seconds(self.rgb.header.stamp)
        if stamp < self.arrival+self.settle*0.5:
            raise RuntimeError(f"latest image {stamp:.2f}s predates arrival {self.arrival:.2f}s (now {self.now():.2f}s)")
        depth = min(self.depths, key=lambda d: abs(M.to_seconds(d.header.stamp)-stamp), default=None)
        if depth is not None and abs(M.to_seconds(depth.header.stamp)-stamp) > 0.1:
            depth = None  # do not localize with a depth frame from another moment
        return self.rgb, depth

    def request_locate(self, target):
        try:
            rgb_msg, depth_msg = self.latest_frames()
        except RuntimeError as exc:
            self.get_logger().warn(f"waiting for a fresh frame: {exc}")
            return  # stay in settling; the next timer tick retries
        rgb = M.image_to_numpy(rgb_msg)
        self.located_frames = (rgb_msg, depth_msg)
        self.state = "grounding"
        self.feedback("grounding", target=target, view_yaw_offset_deg=self.views[0] if self.views else None)
        stamp = M.to_seconds(rgb_msg.header.stamp)
        self.job_kind = "locate"
        self.job = self.pool.submit(lambda: (*self.vlm.locate(rgb, target), stamp))

    def on_located(self, box, image_stamp, call):
        if box is not None:
            located = self.box_to_map(box, *self.located_frames)
            if located is not None:
                point, near = located
                self.target_xy = [round(float(point[0]), 3), round(float(point[1]), 3)]
                x, y, yaw = self.pose
                delta = np.asarray(point[:2])-np.array([x, y])
                distance = float(np.linalg.norm(delta))
                bearing = math.atan2(delta[1], delta[0])
                # Stand-off is measured to the nearest visible surface of the object, not its centre.
                near_distance = float(np.linalg.norm(np.asarray(near[:2])-np.array([x, y])))
                self.feedback("target_located", bbox=[round(v, 1) for v in box], target_xy=self.target_xy,
                              distance_m=round(distance, 2), near_surface_m=round(near_distance, 2),
                              vlm_latency_s=round(call.latency_s, 2))
                # Move closer only when the target looks small; semantic approach poses are chosen
                # for a good view, and walking up to a large target (a dock, a table) loses context.
                height, width = self.info.height or 480, self.info.width or 640
                small = (box[2]-box[0]) < 0.25*width and (box[3]-box[1]) < 0.25*height
                if small and near_distance > self.standoff+0.5:
                    step = near_distance-self.standoff
                    gx, gy = x+step*math.cos(bearing), y+step*math.sin(bearing)
                    self.feedback("refining", goal=[round(gx, 3), round(gy, 3), round(bearing, 3)])
                    self.send_goal(gx, gy, bearing, "refine")
                elif abs(math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))) > math.radians(12):
                    self.send_goal(x, y, bearing, "refine")
                else:
                    self.request_inspect()
                return
            # Visible but not localizable (no depth/TF): inspect from here.
            self.feedback("target_unlocalized", bbox=[round(v, 1) for v in box])
            self.request_inspect()
            return
        # Not visible: look around from the arrival pose before inspecting anyway.
        self.views.pop(0)
        if self.views:
            x, y, _ = self.pose
            sg = self.task.subgoals[-1]
            base = sg.yaw if sg.yaw is not None else self.pose[2]
            self.feedback("looking_around", view_yaw_offset_deg=self.views[0])
            self.send_goal(x, y, base+math.radians(self.views[0]), "look")
        else:
            # Not found from any view: face the place again (its approach yaw) and answer from there.
            self.feedback("target_not_found", target=self.meta.get("target"))
            sg = self.task.subgoals[-1]
            x, y, _ = self.pose
            self.send_goal(x, y, sg.yaw if sg.yaw is not None else self.pose[2], "inspect")

    def box_to_map(self, box, rgb_msg, depth_msg):
        """Map-frame (centre point, nearest-surface point) of a VLM box using aligned depth + TF."""
        if depth_msg is None or self.info is None:
            return None
        depth = M.depth_to_metres(depth_msg)
        x1, y1, x2, y2 = box
        cx, cy, w, h = (x1+x2)/2, (y1+y2)/2, x2-x1, y2-y1
        u0, u1 = int(max(0, cx-0.2*w)), int(min(depth.shape[1], cx+0.2*w+1))
        v0, v1 = int(max(0, cy-0.2*h)), int(min(depth.shape[0], cy+0.2*h+1))
        patch = depth[v0:v1, u0:u1]
        valid = patch[patch > 0.1]
        if valid.size < 5:
            return None
        fx, fy, ppx, ppy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        try:  # exact stamp first, then the latest transform (robot is standing still here)
            tf = self.buffer.lookup_transform("map", depth_msg.header.frame_id, Time.from_msg(depth_msg.header.stamp))
        except Exception:
            try:
                tf = self.buffer.lookup_transform("map", depth_msg.header.frame_id, Time())
            except Exception as exc:
                self.get_logger().warn(f"TF for target localization unavailable: {exc}")
                return None
        matrix = M.transform_matrix(tf.transform)
        points = []
        for z in (float(np.median(valid)), float(np.percentile(valid, 10))):
            points.append((matrix@np.array([(cx-ppx)*z/fx, (cy-ppy)*z/fy, z, 1.0]))[:3])
        return points

    def request_inspect(self):
        try:
            rgb_msg, _ = self.latest_frames()
        except RuntimeError as exc:
            self.get_logger().warn(f"waiting for a fresh frame: {exc}")
            self.state, self.phase = "settling", "inspect"
            return
        rgb = M.image_to_numpy(rgb_msg)
        jpeg = encode_jpeg(rgb, 85)
        self.state = "inspecting"
        self.feedback("inspecting", question=self.meta.get("question"))
        instruction = self.task.instruction or self.instruction
        question, language = self.meta.get("question", ""), self.meta.get("language", "")
        place = self.meta.get("place") or (self.task.subgoals[-1].label if getattr(self.task, "subgoals", None) else "")
        self.job_kind = "inspect"
        self.job = self.pool.submit(lambda: (*self.vlm.inspect(rgb, instruction, question, language, place), jpeg))

    def fail(self, message, **extra):
        self.state = "failed"
        self.feedback("failed", message=message, vlm_calls=self.vlm_log, **extra)


def main():
    rclpy.init()
    node = VLNAgentNode()
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
