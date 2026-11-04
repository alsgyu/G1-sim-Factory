"""Live minimap of a naive-pp task as a web page: the robot's map with room names, the robot where the localizer puts
it, the plan, and the target room. No ROS here: naive_pp.ros.minimap_node (live) and scripts/minimap_replay.py
(a recorded run) feed it.

    http://<this computer or the robot>:8765    open in any browser; it polls the state 4 times a second

Serves
  /            minimap.html
  /map.png     the package's occupancy grid, free space tinted by room (row 0 of the image = highest y)
  /meta.json   map geometry, places (label, Korean alias, rects, center), doorways, anchors
  /state.json  robot pose and current place, trail, plan, goal, target, route, task state, recent feedback
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import re
import threading

import numpy as np
from PIL import Image

from naive_pp.package import FREE, OCCUPIED, MapPackage

PAGE = Path(__file__).with_name("minimap.html")
ROOM_COLORS = [(78, 121, 167), (242, 142, 43), (225, 87, 89), (118, 183, 178), (89, 161, 79), (237, 201, 72),
               (176, 122, 161), (255, 157, 167), (156, 117, 95), (186, 176, 172)]
CORRIDOR_COLOR = (141, 160, 182)
TERMINAL = ("done", "failed", "canceled")


def hangul_alias(place: dict) -> str:
    return next((a for a in place.get("aliases", []) if re.search("[가-힣]", a)), "")


def describe(e: dict) -> str:
    """One line of a feedback event for the minimap's log."""
    s = e.get("state")
    if s == "accepted":
        t = e.get("target", {})
        via = f", 경유 {', '.join(e['waypoints'])}" if e.get("waypoints") else ""
        return f"{t.get('label')} ({e.get('match')}: '{e.get('matched')}'){via}"
    if s in ("planned", "replanned"):
        return f"{e.get('route_text') or e.get('target', {}).get('label')}, {e.get('length_m')} m"
    if s == "progress":
        return f"남은 {e.get('remaining_m')} m, {e.get('place')}"
    if s == "done":
        return f"{e.get('place')}, 목표점까지 {e.get('goal_distance_m')} m, {e.get('elapsed_s')} s"
    return str(e.get("reason") or "")


class Minimap:
    def __init__(self, pkg: MapPackage, trail_spacing: float = 0.25, max_trail: int = 4000, max_events: int = 14):
        self.pkg = pkg
        self.trail_spacing = trail_spacing
        self.max_trail = max_trail
        self.max_events = max_events
        self.lock = threading.Lock()
        colors, places, rooms = [], [], 0
        for p in pkg.places:
            room = p["kind"] == "zone"
            colors.append(ROOM_COLORS[rooms % len(ROOM_COLORS)] if room else CORRIDOR_COLOR)
            rooms += room
            places.append({"id": p["id"], "label": p["label"], "alias": hangul_alias(p),
                           "kind": "room" if room else "corridor", "center": p.get("center"),
                           "rects": p.get("rects", []), "color": "#%02x%02x%02x" % colors[-1]})
        self.anchor_place = {a["id"]: a.get("place") for a in pkg.semantic.get("anchors", [])}
        h, w = pkg.grid.shape
        self.meta = {"package": pkg.folder.name, "origin": list(pkg.origin), "resolution": pkg.resolution,
                     "width": w, "height": h, "places": places,
                     "doors": [e["door"] for e in pkg.semantic.get("edges", []) if e.get("door")],
                     "anchors": [{"id": a["id"], "label": a["label"], "xy": a["xy"], "place": a.get("place")}
                                 for a in pkg.semantic.get("anchors", [])]}
        self.meta_json = json.dumps(self.meta, ensure_ascii=False).encode()
        self.png = self.render(colors)
        self.now = None
        self.pose = None
        self.localization = None
        self.events = []
        self.reset_task()

    def render(self, colors) -> bytes:
        grid = self.pkg.grid
        rgb = np.full(grid.shape+(3,), (214, 214, 210), np.uint8)  # unknown
        rgb[grid == FREE] = (250, 250, 248)
        rgb[grid == OCCUPIED] = (52, 56, 64)
        for k, color in enumerate(colors):
            cells = self.pkg.labels == k
            rgb[cells] = (rgb[cells]*0.72+np.array(color)*0.28).astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(rgb[::-1]).save(buf, "PNG")
        return buf.getvalue()

    def reset_task(self):
        self.task_id = None
        self.instruction = None
        self.target = None
        self.waypoints = []
        self.route = []
        self.doors = []
        self.goal = None
        self.plan = None
        self.plan_list = []
        self.trail = []
        self.state = None
        self.started = None
        self.result = None

    def target_place(self, target: dict, goal=None):
        if target["kind"] == "place":
            return target["id"]
        if target["kind"] == "anchor":
            return self.anchor_place.get(target["id"])
        return self.pkg.place_at(goal[:2]) if goal is not None else None

    # ------------------------------------------------------------------ inputs (any thread)
    def set_pose(self, pose, t=None):
        with self.lock:
            self.now = t if t is not None else self.now
            self.pose = None if pose is None else tuple(float(v) for v in pose)
            if self.pose is None or self.state is None or self.state in TERMINAL:
                return
            xy = self.pose[:2]
            if not self.trail or math.dist(self.trail[-1], xy) >= self.trail_spacing:
                self.trail.append([round(xy[0], 2), round(xy[1], 2)])
                del self.trail[:-self.max_trail]

    def set_plan(self, points):
        points = np.asarray(points, float).reshape(-1, 2)
        if len(points) < 2:  # a one-pose path is naive_pp holding the robot still, not a route
            return
        with self.lock:
            self.plan = points
            self.plan_list = points.round(2).tolist()

    def set_localization(self, state):
        with self.lock:
            self.localization = state

    def on_instruction(self, text: str, t=None):
        with self.lock:
            self.now = t if t is not None else self.now
            self.log({"state": "instruction", "sim_time_s": t, "reason": text[:300]})

    def on_feedback(self, e: dict):
        s = e.get("state")
        with self.lock:
            self.now = e.get("sim_time_s", self.now)
            if s == "accepted":
                self.reset_task()
                self.task_id = e.get("task_id")
                self.instruction = e.get("instruction")
                self.target = {**e["target"], "match": e.get("match"), "matched": e.get("matched")}
                self.target["place"] = self.target_place(self.target)
                self.waypoints = e.get("waypoints", [])
                self.started = e.get("sim_time_s")
                self.state = s
            elif e.get("task_id") != self.task_id or self.target is None:
                # an instruction naive_pp could not read; a running task goes on
                if self.state is None or self.state in TERMINAL:
                    self.state, self.result = s, {"state": s, "reason": e.get("reason")}
            elif s in ("planned", "replanned"):
                self.goal = e.get("goal")
                self.route = e.get("route_places") or [p["id"] for label in e.get("route") or []
                                                       for p in self.pkg.places if p["label"] == label]
                self.doors = e.get("doors", [])
                if self.target["kind"] == "pose":
                    self.target["place"] = self.target_place(self.target, self.goal)
                self.state = s
            elif s == "progress":
                self.state = "following"
            elif s in TERMINAL:
                self.state = s
                self.result = {k: e.get(k) for k in ("state", "place", "in_target_place", "goal_distance_m",
                                                     "elapsed_s", "travelled_m", "replans", "reason")}
            else:
                self.state = s
            if s != "progress":
                self.log(e)

    def log(self, e):
        self.events.append({"t": e.get("sim_time_s"), "state": e.get("state"), "text": describe(e)})
        del self.events[:-self.max_events]

    # ------------------------------------------------------------------ output
    def remaining(self):
        if self.plan is None or self.pose is None:
            return None
        d = np.hypot(*(self.plan-self.pose[:2]).T)
        k = int(np.argmin(d))
        return float(d[k]+np.hypot(*np.diff(self.plan[k:], axis=0).T).sum())

    def snapshot(self) -> dict:
        with self.lock:
            place = self.pkg.place_at(self.pose[:2]) if self.pose is not None else None
            remaining = self.remaining() if self.state not in TERMINAL else None
            return {"t": self.now, "pose": self.pose and [round(v, 3) for v in self.pose],
                    "place": place, "localization": self.localization, "task_id": self.task_id,
                    "instruction": self.instruction, "target": self.target and dict(self.target),
                    "waypoints": self.waypoints, "route": self.route, "doors": self.doors, "goal": self.goal,
                    "plan": self.plan_list, "trail": list(self.trail), "state": self.state, "result": self.result,
                    "remaining_m": None if remaining is None else round(remaining, 1),
                    "elapsed_s": None if self.started is None or self.now is None else
                    round(self.now-self.started, 1), "events": list(self.events)}


def serve(view: Minimap, host: str = "0.0.0.0", port: int = 8765) -> ThreadingHTTPServer:
    """Serve the minimap from a background thread; returns the server (shutdown() to stop)."""
    page = PAGE.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                body, kind = page, "text/html; charset=utf-8"
            elif path == "/map.png":
                body, kind = view.png, "image/png"
            elif path == "/meta.json":
                body, kind = view.meta_json, "application/json; charset=utf-8"
            elif path == "/state.json":
                body, kind = json.dumps(view.snapshot(), ensure_ascii=False).encode(), "application/json; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name="minimap-http", daemon=True).start()
    return server
