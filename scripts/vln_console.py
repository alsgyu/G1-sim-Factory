#!/usr/bin/env python3
"""Interactive external-agent console: type instructions, watch the VLN decide, get the result.

    bash scripts/vln_console.sh                 # starts the VLN stack and this console (Isaac Sim running)
    bash scripts/g1_docker.sh python3 scripts/vln_console.py   # console only (stack already running)

At the prompt type an instruction in Korean or English, or a command:
  :where   current pose and zone        :tasks   example instructions       :cancel  stop the robot
  :help    this help                    :quit    leave (the robot stays where it is)
Every instruction gets a folder outputs/console/<time>_<words>/ with feedback.jsonl, decisions/*.jpg (what the
VLM saw), inspection.jpg (evidence), summary.json and route.png (decisions on the map). The robot starts each
instruction where the previous one ended.
"""
from __future__ import annotations

import json
from pathlib import Path
import re
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory.ros2.frames import MapPose  # noqa: E402
from g1_factory.ros2.qos import TOPICS  # noqa: E402

BOLD, DIM, GREEN, RED, CYAN, YELLOW, RESET = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[36m", "\033[33m", "\033[0m"
ACTIONS = {"move": "이동", "turn": "회전·둘러보기", "approach": "접근", "stop": "정지"}


def load_zones():
    for path in (ROOT/"outputs/isaac/campus_meta.json",):
        if path.is_file():
            meta = json.loads(path.read_text())
            return {zid: (z["label"], z["bounds"]) for zid, z in meta["zones"].items()}
    from g1_factory.scene import load_config
    return {z["id"]: (z["label"], z["bounds"]) for z in load_config(ROOT/"configs/factory.yaml")["zones"]}


def zone_name(zones, x, y):
    for label, (x0, y0, x1, y1) in zones.values():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return label
    return "aisle / corridor"


class Console(Node):
    def __init__(self):
        super().__init__("vln_console")
        sub = lambda name, msg, cb: self.create_subscription(msg, name, cb, TOPICS[name][1])
        self.pub = self.create_publisher(String, "/vln/instruction", TOPICS["/vln/instruction"][1])
        sub("/vln/feedback", String, self.on_feedback)
        sub("/vln/inspection_image", CompressedImage, self.on_inspection)
        sub("/vln/decision_image", CompressedImage, self.on_decision_image)
        sub("/odom", Odometry, self.on_odom)
        # A map package (map_file node): rooms in the frame the VLN runs in; its own map frame means the simulator
        # campus is not the right background for route.png.
        self.create_subscription(String, "/slam/semantic_map", self.on_package, TOPICS["/semantic_map"][1])
        self.map_pose = MapPose(self)  # the map frame the VLN plans in (map -> odom: simulator or localizer)
        self.zones, self.package_frame = None, False
        self.pose, self.run, self.lock = None, None, threading.Lock()
        self.finished = threading.Event()
        self.decision_count = 0

    # ---------------------------------------------------------------- run folder
    def start(self, text):
        slug = re.sub(r"[^0-9A-Za-z가-힣]+", "_", text)[:40].strip("_") or "instruction"
        folder = ROOT/"outputs/console"/f"{time.strftime('%Y%m%d-%H%M%S')}_{slug}"
        (folder/"decisions").mkdir(parents=True, exist_ok=True)
        (folder/"instruction.txt").write_text(text+"\n", encoding="utf-8")
        with self.lock:
            self.run = {"folder": folder, "instruction": text, "events": [], "start_pose": self.pose,
                        "images": 0}
        self.finished.clear()
        self.decision_count = 0
        self.pub.publish(String(data=text))
        return folder

    def on_odom(self, msg):
        pose = self.map_pose(msg)
        self.pose = list(pose) if pose else self.pose

    def on_package(self, msg):
        semantic = json.loads(msg.data)
        self.zones = {z["id"]: (z["label"], z["bounds"]) for z in semantic["zones"]}
        self.package_frame = semantic.get("frame") == "map"

    def on_decision_image(self, msg):
        with self.lock:
            if self.run:
                self.run["images"] += 1
                (self.run["folder"]/"decisions"/f"{self.run['images']-1:03d}.jpg").write_bytes(bytes(msg.data))

    def on_inspection(self, msg):
        with self.lock:
            if self.run:
                (self.run["folder"]/"inspection.jpg").write_bytes(bytes(msg.data))

    def on_feedback(self, msg):
        event = json.loads(msg.data)
        with self.lock:
            if not self.run:
                return
            self.run["events"].append(event)
            with (self.run["folder"]/"feedback.jsonl").open("a", encoding="utf-8") as f:
                f.write(json.dumps(event, ensure_ascii=False)+"\n")
        self.show(event)
        if event.get("state") in ("done", "failed", "canceled", "preempted"):
            self.finish(event)

    # ---------------------------------------------------------------- display
    def show(self, e):
        t, state = e.get("sim_time_s", 0.0), e.get("state")
        head = f"{DIM}[{t:7.1f} s]{RESET}"
        if state == "planned":
            steps = " → ".join(f"{s['goal']}" + (f" ({s['region']})" if s.get("region") else "")
                               for s in e.get("steps") or [])
            label = f", 명판 {e['target_label']}" if e.get("target_label") else ""
            print(f"{head} {BOLD}계획{RESET} {steps}  | 대상: {e.get('target')}{label}", flush=True)
        elif state == "decision":
            self.decision_count += 1
            act = ACTIONS.get(e.get("action"), e.get("action"))
            extra = f" #{e['waypoint']}" if e.get("waypoint") else ""
            plate = f" ({e['plate']})" if e.get("plate") and len(e["plate"]) <= 20 else ""
            note = f" {DIM}[{e['note']}]{RESET}" if e.get("note") else ""
            print(f"{head} {CYAN}결정 {self.decision_count}{RESET} {BOLD}{act}{extra}{plate}{RESET}{note} — "
                  f"{(e.get('thought') or '')[:150]}", flush=True)
        elif state == "step_done":
            print(f"{head} {GREEN}단계 완료{RESET} → 다음 단계 {e.get('step')}", flush=True)
        elif state == "target_located":
            print(f"{head} {YELLOW}대상 위치{RESET} {e.get('target_xy')} → 접근", flush=True)
        elif state == "arrived":
            print(f"{head} {GREEN}대상 앞 도착{RESET}, 확인 중…", flush=True)
        elif state == "done":
            r = e.get("result") or {}
            print(f"{head} {GREEN}{BOLD}완료{RESET}  판정 {BOLD}{r.get('verdict')}{RESET} — {r.get('answer')}\n"
                  f"           관찰: {r.get('observed')}", flush=True)
        elif state in ("failed", "canceled", "preempted"):
            print(f"{head} {RED}{state}{RESET} {e.get('message', '')}", flush=True)

    def finish(self, event):
        with self.lock:
            run, self.run = self.run, None
        time.sleep(1.5)  # inspection image may follow "done"
        decisions = [e for e in run["events"] if e.get("state") == "decision"]
        start = run["start_pose"] or next((e["pose"] for e in decisions if e.get("pose")), None) or [0, 0, 0]
        summary = {"instruction": run["instruction"], "state": event.get("state"),
                   "result": event.get("result"), "start_pose": start, "final_pose": self.pose or start,
                   "decisions": len(decisions), "target_xy": event.get("target_xy"),
                   "task": run["folder"].name, "expected_place": None}
        (run["folder"]/"summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
        try:
            if self.package_frame:
                raise RuntimeError("the robot runs in the map package's own frame; route.png draws the simulator campus")
            from plot_vln_episode import route_figure
            route_figure(run["folder"])
            print(f"{DIM}  → 결과: {run['folder'].relative_to(ROOT)} (route.png, decisions/, inspection.jpg){RESET}")
        except Exception as exc:  # figures are a convenience; never lose the run because of them
            print(f"{DIM}  → 결과: {run['folder'].relative_to(ROOT)} (route figure skipped: {exc}){RESET}")
        self.finished.set()


def help_text():
    print(__doc__.split("At the prompt")[1].split("Every instruction")[0].strip())


def main():
    sys.path.insert(0, str(ROOT/"scripts"))
    rclpy.init()
    node = Console()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    zones = load_zones()
    print(f"{BOLD}G1 VLN 콘솔{RESET} — 지시를 입력하세요 (:help 도움말, :quit 종료)")
    deadline = time.monotonic()+20
    while node.count_subscribers("/vln/instruction") == 0 and time.monotonic() < deadline:
        time.sleep(0.5)
    if node.count_subscribers("/vln/instruction") == 0:
        print(f"{RED}vln_agent가 보이지 않습니다. VLN 스택(agent:=false)과 Isaac Sim이 실행 중인지 확인하세요.{RESET}")
    try:
        while True:
            try:
                text = input(f"\n{BOLD}지시>{RESET} ").strip()
            except EOFError:
                break
            if not text:
                continue
            if text in (":quit", ":q", ":exit"):
                break
            if text == ":help":
                help_text()
                continue
            if text == ":where":
                if node.pose:
                    x, y, yaw = node.pose
                    print(f"위치 ({x:.2f}, {y:.2f}), 방향 {yaw*57.2958:.0f}°, 구역: {zone_name(node.zones or zones, x, y)}")
                else:
                    print("아직 지도 좌표의 위치가 없습니다(/odom 또는 위치 추정을 기다리는 중).")
                continue
            if text == ":tasks":
                import yaml
                tasks = yaml.safe_load((ROOT/"configs/vln_tasks.yaml").read_text(encoding="utf-8"))["tasks"]
                for name, t in tasks.items():
                    if isinstance(t.get("instruction"), str):
                        print(f"  {name}: {t['instruction']}")
                continue
            if text == ":cancel":
                node.pub.publish(String(data=json.dumps({"cancel": True})))
                print("정지 요청을 보냈습니다.")
                continue
            deadline = time.monotonic()+10
            while node.pose is None and time.monotonic() < deadline:  # start pose for the route figure
                time.sleep(0.2)
            folder = node.start(text)
            print(f"{DIM}  지시 전송 → {folder.relative_to(ROOT)}  (Ctrl-C: 이 지시 취소){RESET}")
            try:
                while not node.finished.wait(0.5):
                    pass
            except KeyboardInterrupt:
                node.pub.publish(String(data=json.dumps({"cancel": True})))
                print(f"\n{YELLOW}취소했습니다.{RESET}")
                node.finished.wait(5)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
