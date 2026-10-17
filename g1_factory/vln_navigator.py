"""Hierarchical VLN high level: map-guided, vision-in-the-loop waypoint selection.

Adapted from MapGPT's map-guided prompting (Chen et al., ACL 2024) to a continuous
factory map and a humanoid, with candidate markers drawn on the camera image
(visual prompting, cf. PIVOT / VLMnav). At every decision the VLM sees:

* the operator instruction and the step plan from layer 1,
* what the annotated SLAM map says: region names with direction/distance, and for
  each candidate how much closer (walking distance) it brings the robot to the
  current step's region,
* reachable waypoints (straight, obstacle-free segments on the SLAM map), the
  ones in view drawn as numbered circles on the current head-camera image,
* its own recent decisions.

It answers with one action: move to waypoint k, turn to look, approach an object it
sees (bounding box -> depth -> stand-off pose), or stop. Fine-grained targets are
not on the map; the robot must find them visually, and when the instruction names a
plate ("STATION 2", "CHARGER 3") an approach is accepted only after the VLM re-reads
that plate in a crop of the image. The output is a choice among map-feasible options
(greedy decoding, validated), so the policy cannot invent motions and identical
inputs give identical decisions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import re

import numpy as np

from .navigation import wrap_angle
from .roadmap import Candidate, RoadMap, compass_name, describe_direction, draw_candidates, project_points, region_at
from .vlm import VLMClient, extract_json

NAV_SYSTEM = """You are the high-level navigation policy (VLN) of a Unitree G1 humanoid robot in a factory \
campus (a warehouse and an office building). You decide where the robot walks next, using the operator \
instruction, the annotated map and the head-camera image. Numbered white circles drawn on the image are \
reachable floor waypoints in view; waypoints outside the view are only listed with their direction. \
Map directions: north = +y, east = +x. Zone signs, name plates (e.g. "CHARGER 3", "STATION 2", \
"CONVEYOR A") and objects in the image tell you where things are; fine-grained targets are not on the map, \
so look for them. For each waypoint you are told how many metres of walking it saves towards the current \
step's region (negative = it leads away); use it to travel between regions, use the image to find objects.
Choose exactly one action:
- "move": walk to waypoint <number>. Prefer waypoints that make progress on the current step; prefer \
unvisited ones when searching.
- "turn": turn "left", "right" or "around" in place to look, when what you need may be outside the view.
- "approach": the object of the current step is clearly visible: give its bounding box [x1, y1, x2, y2] \
normalized to 0-1000 (x right, y down). When the task names a plate ("name plate: ..." below), approach only \
an object whose plate you can read and that shows the required number or letter (do not guess from shape). Once the object is in view, \
approach it instead of walking past it.
- "stop": a travel step is complete (you are in its region). The LAST step ("find ... and stand in front of it") \
is never completed by "stop" from a distance: approach the object (bounding box) - the robot then walks up to it.
When you are in the region where the target should be but cannot see it, turn to look before walking away. \
Do not repeat "approach" on an object you already stand in front of: then "stop".
Reply with one JSON object and nothing else:
{"thought": "<one short sentence>", "action": "move|turn|approach|stop", "waypoint": <number or null>, \
"turn": "left|right|around|null", "bbox_2d": [x1, y1, x2, y2] or null}"""

DECOMPOSE_SYSTEM = """You plan for the navigation policy of a factory robot. Break the operator's \
instruction into 1-5 navigation steps. Each step has a short English `goal` (where to go or what to find) \
and the annotated `region` it happens in (one of the region ids listed, or null). The last step's goal must \
be "find <the object to check> and stand in front of it". Also give:
- `target`: the fixed object the camera must look at to answer (e.g. "charging dock 3" for "is a cart \
parked at dock 3?"; never an object whose presence is being asked about). Keep the words the instruction uses \
to single it out: "the free charging dock (no cart parked)", "the red fire extinguisher cabinet", "the blue \
electrical panel on the east wall",
- `target_label`: the exact text on that object's name plate when the instruction identifies it by a number, \
an ordinal or a name (equipment plates read like "STATION 2", "CHARGER 3", "CONVEYOR A", "WORKBENCH 1", \
"SERVER RACK 1", "FIRE EXTINGUISHER"); "second station" -> "STATION 2". Use "" when there is no such plate,
- `question`: the check, in the instruction's language.
Reply with one JSON object:
{"steps": [{"goal": "...", "region": "<region id or null>"}], "target": "...", "target_label": "...", \
"question": "...", "language": "ko|en"}"""

SCAN_SYSTEM = """You are the high-level navigation policy (VLN) of a Unitree G1 humanoid robot in a factory \
campus. The robot has just reached the region where the object of the current step should be and has looked \
around from one spot: you get four head-camera views, in order view 1 to view 4, each facing the compass \
direction listed. Pick the view the robot should turn to and continue from: the view in which the object is \
visible (confirm it by its name plate when it has one; do not guess from shape), otherwise the most promising \
direction to search (open floor towards parts of the region not yet seen). Reply with one JSON object and \
nothing else: {"seen": "<a few words per view>", "view": <1-4>, "target_visible": true|false}"""

VERIFY_SYSTEM = """You double-check targets for a robot. The image is a close-up of the object the robot wants \
to walk to (in the centre) with the floor in front of it. Say what the object is and whether it is the requested \
target. The request may include a condition ("free", "empty", "red", "with a warning sign", a number): it is the \
target only if it is the right kind of object AND the condition visibly holds. Reply with one JSON object and \
nothing else: {"what": "<what the object is and its condition, a few words>", "is_target": true|false}"""

READ_SYSTEM = """You read name plates for a robot. Look at the object in the image and the sign or plate \
on or just above it. Reply with one JSON object and nothing else: {"text": "<plate text exactly as written, \
or empty if none is readable>"}"""


@dataclass
class NavStep:
    goal: str
    region: str | None = None


@dataclass
class NavTask:
    instruction: str
    steps: list
    target: str
    question: str
    language: str = ""
    target_label: str = ""
    index: int = 0

    @property
    def current(self) -> NavStep | None:
        return self.steps[self.index] if self.index < len(self.steps) else None


@dataclass
class Decision:
    action: str                      # move | turn | approach | stop | finished | failed
    goal: tuple | None = None        # (x, y, yaw) for move/turn
    waypoint: Candidate | None = None
    bbox: tuple | None = None        # pixels, for approach
    thought: str = ""
    raw: str = ""
    latency_s: float = 0.0
    request_bytes: int = 0
    prompt_image: np.ndarray | None = None
    note: str = ""
    plate: str | None = None         # text read on the approached object, when checked


def _normalize_label(text: str) -> str:
    return re.sub(r"[^A-Z0-9]+", " ", text.upper()).strip()


_ORDINALS = {
    "1": ("1", "첫", "하나", "first", "one"), "2": ("2", "두 번째", "둘째", "두번째", "second", "two"),
    "3": ("3", "세 번째", "셋째", "세번째", "third", "three"), "4": ("4", "네 번째", "넷째", "네번째", "fourth", "four"),
    "5": ("5", "다섯 번째", "다섯째", "fifth", "five"),
}


def plate_ids(text: str) -> set:
    """Identifier tokens of a plate text: numbers and single letters ("STATION 2" -> {"2"})."""
    return {t for t in _normalize_label(text).split() if t.isdigit() or len(t) == 1}


def plate_matches(label: str, plate: str) -> bool:
    """The plate carries the instruction's identifier. The kind of object ("STATION" vs "VISION INSPECTION")
    is the VLM's visual judgement; this check tells STATION 2 from STATION 1/3/4."""
    ids = plate_ids(label)
    return bool(ids) and ids <= plate_ids(plate)


def plate_hint(label: str) -> str:
    """How the prompt names the plate to look for: the identifier is what must match."""
    if not label:
        return ""
    return f' (name plate: "{label}"; the plate must show {" ".join(sorted(plate_ids(label)))})'


def grounded_label(label: str, instruction: str) -> str:
    """Keep a plate label only if the instruction itself names its identifier.

    "second inspection station" -> "STATION 2" is kept; "the conveyor" -> "CONVEYOR A"
    is dropped, because the operator did not say which conveyor (a model guess must
    not become a hard constraint).
    """
    label = _normalize_label(label)
    if not label:
        return ""
    text = instruction.lower()
    ids = [t for t in label.split() if t.isdigit() or len(t) == 1]
    if not ids:
        return ""
    for token in ids:
        words = _ORDINALS.get(token, (token.lower(),))
        if not any(re.search(rf"(?<![a-z0-9]){re.escape(w)}(?![a-z0-9])", text) for w in words):
            return ""
    return label


def decompose(client: VLMClient, instruction: str, regions: dict, labels: dict) -> tuple[NavTask, dict]:
    """Layer 1: instruction -> step plan over annotated regions (validated, one retry)."""
    region_text = "\n".join(f"- {rid}: {labels.get(rid, rid)}" for rid in regions)
    prompt = f"Annotated regions:\n{region_text}\n\nInstruction: {instruction}"
    error = None
    for attempt in range(2):
        text = prompt if error is None else prompt+f"\n\nYour previous reply was invalid: {error}."
        call = client.chat(DECOMPOSE_SYSTEM, text, max_tokens=400)
        try:
            reply = extract_json(call.text)
            steps = reply.get("steps")
            if not isinstance(steps, list) or not 1 <= len(steps) <= 5:
                raise ValueError("steps must be a list of 1-5 items")
            parsed = []
            for step in steps:
                goal = str(step.get("goal", "")).strip() if isinstance(step, dict) else ""
                region = step.get("region") if isinstance(step, dict) else None
                if not goal:
                    raise ValueError("every step needs a goal")
                if region in ("null", "", "none", None):
                    region = None
                elif region not in regions:
                    raise ValueError(f"unknown region {region!r}")
                parsed.append(NavStep(goal, region))
            target = str(reply.get("target", "")).strip() or parsed[-1].goal
            raw_label = str(reply.get("target_label", ""))
            label = grounded_label(raw_label, instruction)
            if raw_label and not label:  # a guessed number ("server rack 1") must not narrow the search either
                for token in plate_ids(raw_label):
                    # letters are plate ids only in capitals: "conveyor A", never the article in "a conveyor"
                    unguess = lambda text: re.sub(rf"(\b[A-Za-z]+)\s+{re.escape(token)}\b", r"\1", text)
                    target = unguess(target)
                    parsed = [NavStep(unguess(step.goal), step.region) for step in parsed]
            if not re.search(r"\bfind\b", parsed[-1].goal, re.IGNORECASE):
                if attempt == 0:
                    raise ValueError('the last step must be "find <the object to check> and stand in front of it"')
                if len(parsed) == 5:
                    parsed.pop(-2)
                parsed.append(NavStep(f"find {target} and stand in front of it", parsed[-1].region))
            task = NavTask(instruction, parsed, target, str(reply.get("question", "")).strip() or instruction,
                           str(reply.get("language", "")), label)
            return task, {"latency_s": call.latency_s, "request_bytes": call.request_bytes,
                          "response_bytes": call.response_bytes, "attempts": attempt+1, "raw": call.text}
        except ValueError as exc:
            error = str(exc)
    raise ValueError(f"instruction decomposition failed: {error}")


@dataclass
class VLNNavigator:
    client: VLMClient
    roadmap: RoadMap
    region_labels: dict                  # region id -> human label
    region_entrances: dict               # region id -> (x, y) annotated entrance point
    anchors: dict = field(default_factory=dict)   # annotated landmark label -> (x, y)
    max_decisions: int = 40
    max_turns_in_place: int = 3          # 270 deg of looking around, then the VLN must move
    history: list = field(default_factory=list)
    visited: list = field(default_factory=list)
    task: NavTask | None = None
    decisions: int = 0
    turns_here: int = 0                  # in-place turns since the last move/approach (not position-based:
                                         # a walking humanoid drifts while turning on the spot)
    scanned: int = -1                    # step index whose region was already looked around in

    def start(self, task: NavTask):
        self.task, self.history, self.visited, self.decisions = task, [], [], 0
        self.turns_here, self.scanned = 0, -1

    def note_position(self, xy):
        if not self.visited or np.linalg.norm(np.asarray(self.visited[-1])-xy) > 0.8:
            self.visited.append([float(xy[0]), float(xy[1])])

    def auto_advance(self, xy) -> bool:
        """A region step is complete as soon as the robot is inside that annotated region."""
        step = self.task.current if self.task else None
        if step and step.region and self.task.index < len(self.task.steps)-1 and \
                region_at(self.roadmap.regions, xy) == step.region:
            self.history.append(f"step {self.task.index+1} done (entered {self.region_labels.get(step.region)})")
            self.task.index += 1
            return True
        return False

    def needs_scan(self, xy) -> bool:
        """First time inside the region of the final (find-the-object) step: look around before deciding.

        The head camera sees 69 deg; a target beside the robot is otherwise never in view. This is the
        panoramic observation of R2R-style VLN (MapGPT prompts with every direction), taken once per region.
        """
        t = self.task
        step = t.current if t else None
        return (step is not None and t.index == len(t.steps)-1 and step.region is not None
                and self.scanned != t.index and region_at(self.roadmap.regions, xy) == step.region)

    def choose_view(self, xy, views) -> Decision:
        """One VLM choice among looked-around views [(rgb, yaw)] taken from the same spot -> turn to it."""
        t = self.task
        self.scanned = t.index
        if self.decisions >= self.max_decisions:
            return Decision("failed", note="decision limit reached")
        self.decisions += 1
        label = plate_hint(t.target_label)
        listing = "\n".join(f"  view {k+1}: facing {compass_name(yaw)}" for k, (_, yaw) in enumerate(views))
        text = (f"Instruction: {t.instruction}\nCurrent step: {t.current.goal}\nObject to find: {t.target}{label}\n"
                f"Views from where you stand:\n{listing}\nWhich view should the robot continue from?")
        error, latency, request_bytes, raw, reply, view = None, 0.0, 0, "", {}, None
        for attempt in range(2):
            try:
                call = self.client.chat(SCAN_SYSTEM, text if error is None else
                                        text+f"\n\nYour previous reply was rejected: {error}",
                                        [rgb for rgb, _ in views], max_tokens=200)
            except Exception as exc:  # e.g. the server's context is too small for four images: keep navigating
                error, view = f"look-around call failed: {type(exc).__name__}: {exc}", None
                break
            latency, request_bytes, raw = latency+call.latency_s, request_bytes+call.request_bytes, call.text
            try:
                reply = extract_json(call.text)
                view = int(reply.get("view"))
                if not 1 <= view <= len(views):
                    raise ValueError(f"view must be a number from 1 to {len(views)}")
                break
            except (ValueError, TypeError) as exc:
                error, view = str(exc), None
        note = f"looked around: view {view} of {len(views)}"
        if view is None:
            view, note = 1, f"no valid reply ({error}); continuing from view 1"
        heading = views[view-1][1]
        visible = bool(reply.get("target_visible")) if reply else False
        self.history.append(f"looked around here and turned to view {view} ({compass_name(heading)})"
                            + (", the target is in that view" if visible else ""))
        self.turns_here = 0
        return Decision("turn", (float(xy[0]), float(xy[1]), heading), thought=str(reply.get("seen", ""))[:300],
                        raw=raw, latency_s=latency, request_bytes=request_bytes, note=note,
                        prompt_image=view_mosaic(views))

    def map_context(self, xy, yaw) -> str:
        lines = []
        for rid, (ex, ey) in self.region_entrances.items():
            d = math.hypot(ex-xy[0], ey-xy[1])
            heading = math.atan2(ey-xy[1], ex-xy[0])
            b = math.degrees(wrap_angle(heading-yaw))
            lines.append(f"- {self.region_labels.get(rid, rid)} [{rid}] entrance: {describe_direction(b)} "
                         f"({b:+.0f} deg, {compass_name(heading)}), {d:.1f} m straight-line")
        for label, (ax, ay) in self.anchors.items():
            d = math.hypot(ax-xy[0], ay-xy[1])
            heading = math.atan2(ay-xy[1], ax-xy[0])
            b = math.degrees(wrap_angle(heading-yaw))
            lines.append(f"- {label}: {describe_direction(b)} ({b:+.0f} deg, {compass_name(heading)}), {d:.1f} m")
        return "\n".join(lines)

    def prompt(self, xy, yaw, candidates) -> str:
        t = self.task
        plan = "\n".join(f"  {k+1}. {s.goal}" + (f" [region: {s.region}]" if s.region else "")
                         + ("  <- current" if k == t.index else "  (done)" if k < t.index else "")
                         for k, s in enumerate(t.steps))
        here = region_at(self.roadmap.regions, xy)
        region = t.current.region if t.current else None
        rows = []
        for c in candidates:
            route = ""
            if region and c.route_gain is not None:
                route = (f", {abs(c.route_gain):.1f} m {'closer to' if c.route_gain >= 0 else 'farther from'} "
                         f"{self.region_labels.get(region, region)}")
            rows.append(f"  {c.id}: {describe_direction(c.bearing_deg)} ({c.bearing_deg:+.0f} deg, {c.compass}), "
                        f"{c.distance:.1f} m, {self.region_labels.get(c.region, c.region)}, "
                        f"{'IN VIEW' if c.pixel else 'not in view'}{', visited' if c.visited else ''}{route}")
        waypoints = "\n".join(rows) or "  (none)"
        history = "\n".join(f"  - {h}" for h in self.history[-8:]) or "  (start)"
        label = plate_hint(t.target_label)
        return (f"Instruction: {t.instruction}\nStep plan:\n{plan}\nTarget to inspect at the end: {t.target}{label}\n\n"
                f"You are in: {self.region_labels.get(here, here)}; you face {compass_name(yaw)}.\n"
                f"Annotated map (from you):\n{self.map_context(xy, yaw)}\n\nReachable waypoints:\n{waypoints}\n\n"
                f"Recent decisions:\n{history}\n\nDecide the next action for the current step.")

    def decide(self, xy, yaw, rgb, map_from_cam, intrinsics) -> Decision:
        """One VLM decision (plus a plate check for approaches). intrinsics = (fx, fy, cx, cy, w, h)."""
        if self.task is None or self.task.current is None:
            return Decision("finished")
        if self.decisions >= self.max_decisions:
            return Decision("failed", note="decision limit reached")
        self.decisions += 1
        candidates = self.roadmap.candidates(xy, yaw, visited=self.visited,
                                             region=self.task.current.region if self.task.current else None)
        floor = np.array([[c.xy[0], c.xy[1], 0.0] for c in candidates]).reshape(-1, 3)
        pixels = project_points(floor, map_from_cam, *intrinsics) if len(candidates) else []
        for c, px in zip(candidates, pixels):
            c.pixel = px
        image = draw_candidates(rgb, candidates)
        text = self.prompt(xy, yaw, candidates)
        error, latency, request_bytes, raw = None, 0.0, 0, ""
        for attempt in range(3):
            call = self.client.chat(NAV_SYSTEM, text if error is None else
                                    text+f"\n\nYour previous reply was rejected: {error}", [image], max_tokens=250)
            latency, request_bytes, raw = latency+call.latency_s, request_bytes+call.request_bytes, call.text
            try:
                decision = self._parse(call.text, candidates, xy, yaw, rgb.shape)
                if decision.action == "approach" and self.task.target_label:
                    plate, read = self.read_plate(rgb, decision.bbox)
                    latency, request_bytes = latency+read.latency_s, request_bytes+read.request_bytes
                    decision.plate = plate
                    if not plate_matches(self.task.target_label, plate):
                        raise ValueError(f'the plate on that object reads "{plate or "nothing readable"}", '
                                         f'not "{self.task.target_label}". Look for the right one.')
                elif decision.action == "approach":
                    what, ok, check = self.verify_target(rgb, decision.bbox)
                    latency, request_bytes = latency+check.latency_s, request_bytes+check.request_bytes
                    decision.plate = what
                    if not ok:
                        raise ValueError(f'a closer look says that object is "{what}", not {self.task.target}. '
                                         f"Keep searching.")
                decision.latency_s, decision.request_bytes, decision.raw = latency, request_bytes, raw
                decision.prompt_image = image
                self._remember(decision)
                return decision
            except ValueError as exc:
                error = str(exc)
                self.history.append(f"rejected: {error}")
        # Deterministic fallback: look around (or leave the spot after a full look-around).
        if self.turns_here < self.max_turns_in_place:
            self.turns_here += 1
            decision = Decision("turn", (float(xy[0]), float(xy[1]), wrap_angle(yaw+math.pi/2)),
                                note=f"no valid reply ({error}); turning left to look")
        else:
            unvisited = [c for c in candidates if not c.visited] or candidates
            if not unvisited:
                return Decision("failed", note="no reachable waypoint")
            c = max(unvisited, key=lambda c: (c.route_gain or 0.0, c.distance))
            decision = Decision("move", (float(c.xy[0]), float(c.xy[1]), math.atan2(c.xy[1]-xy[1], c.xy[0]-xy[0])),
                                waypoint=c, note=f"no valid reply ({error}); moving on")
        decision.raw, decision.latency_s, decision.request_bytes, decision.prompt_image = raw, latency, request_bytes, image
        self._remember(decision)
        return decision

    def crop(self, rgb, box, up=0.4, side=0.3, down=0.2):
        from PIL import Image
        h, w = rgb.shape[:2]
        x1, y1, x2, y2 = box
        bw, bh = x2-x1, y2-y1
        crop = rgb[int(max(0, y1-up*bh)):int(min(h, y2+down*bh)), int(max(0, x1-side*bw)):int(min(w, x2+side*bw))]
        if crop.size == 0:
            crop = rgb
        return np.asarray(Image.fromarray(crop).resize((max(96, crop.shape[1]*2), max(96, crop.shape[0]*2))))

    def verify_target(self, rgb, box):
        """Second look at an unlabelled target in a crop: cuts approaches to look-alikes."""
        call = self.client.chat(VERIFY_SYSTEM, f"Requested target: {self.task.target}. Is this object it?",
                                [self.crop(rgb, box, side=0.5, down=1.0)], max_tokens=80)
        try:
            reply = extract_json(call.text)
            return str(reply.get("what", "")), bool(reply.get("is_target")), call
        except ValueError:
            return call.text.strip()[:80], False, call

    def read_plate(self, rgb, box):
        """Crop the object (and the space above it, where plates hang) and read its plate."""
        call = self.client.chat(READ_SYSTEM, "What does the name plate of this object say?",
                                [self.crop(rgb, box, up=1.5, side=0.6)], max_tokens=60)
        try:
            text = str(extract_json(call.text).get("text", ""))
        except ValueError:
            text = call.text.strip()
        return text, call

    def _parse(self, text, candidates, xy, yaw, shape) -> Decision:
        reply = extract_json(text)
        action = str(reply.get("action", "")).lower().strip()
        thought = str(reply.get("thought", ""))[:300]
        if action == "move":
            try:
                k = int(reply.get("waypoint"))
            except (TypeError, ValueError):
                raise ValueError("move needs a waypoint number") from None
            match = [c for c in candidates if c.id == k]
            if not match:
                raise ValueError(f"waypoint {k} is not one of {[c.id for c in candidates]}")
            c = match[0]
            self._check_route(c, candidates, xy)
            heading = math.atan2(c.xy[1]-xy[1], c.xy[0]-xy[0])
            return Decision("move", (float(c.xy[0]), float(c.xy[1]), heading), waypoint=c, thought=thought)
        if action == "turn":
            if self.turns_here >= self.max_turns_in_place:
                raise ValueError("you have already looked around here; move to a waypoint (prefer unvisited ones "
                                 "that bring you closer)")
            turn = str(reply.get("turn", "")).lower()
            delta = {"left": math.pi/2, "right": -math.pi/2, "around": math.pi}.get(turn)
            if delta is None:
                raise ValueError("turn must be left, right or around")
            self.turns_here += 1
            return Decision("turn", (float(xy[0]), float(xy[1]), wrap_angle(yaw+delta)), thought=thought)
        if action == "approach":
            box = reply.get("bbox_2d")
            if not isinstance(box, list) or len(box) != 4:
                raise ValueError("approach needs bbox_2d [x1, y1, x2, y2]")
            x1, y1, x2, y2 = (float(v) for v in box)
            x1, x2 = sorted((x1, x2))
            y1, y2 = sorted((y1, y2))
            if not (0 <= x1 < x2 <= 1000 and 0 <= y1 < y2 <= 1000):
                raise ValueError("bbox_2d must be inside 0-1000")
            h, w = shape[:2]
            return Decision("approach", bbox=(x1*w/1000, y1*h/1000, x2*w/1000, y2*h/1000), thought=thought)
        if action == "stop":
            if self.task.index == len(self.task.steps)-1:
                raise ValueError("this is the last step: the robot must stand in front of the object, so do not "
                                 "stop here - if you can see it, answer \"approach\" with its bounding box, "
                                 "otherwise move or turn to find it")
            return Decision("stop", thought=thought)
        raise ValueError(f"unknown action {action!r}")

    def _check_route(self, c: Candidate, candidates, xy):
        """Consistency rules with the annotated map (the VLM still chooses among what passes):
        do not leave the step's region once inside it, and do not walk clearly away from it."""
        region = self.task.current.region if self.task.current else None
        if not region:
            return
        inside = region_at(self.roadmap.regions, xy) == region
        label = self.region_labels.get(region, region)
        if inside and c.region != region and any(o.region == region for o in candidates):
            raise ValueError(f"waypoint {c.id} leaves {label}, where the current step happens; "
                             f"choose a waypoint inside {label} or turn to look")
        if not inside and c.route_gain is not None and c.route_gain < -1.0 and \
                any((o.route_gain or 0) > 0 for o in candidates):
            raise ValueError(f"waypoint {c.id} leads {abs(c.route_gain):.1f} m away from {label}; "
                             f"choose one that brings you closer")

    def _remember(self, d: Decision):
        if d.action in ("move", "approach", "stop"):
            self.turns_here = 0
        if d.action == "move":
            self.history.append(f"move {d.waypoint.compass} {d.waypoint.distance:.1f} m into "
                                f"{self.region_labels.get(d.waypoint.region, d.waypoint.region)}"
                                + (f" ({d.note})" if d.note else ""))
        elif d.action == "turn":
            self.history.append("turn to look" + (f" ({d.note})" if d.note else ""))
        elif d.action == "approach":
            self.history.append("approach the visible target" + (f' (plate "{d.plate}")' if d.plate else ""))
        elif d.action == "stop":
            self.history.append(f"step {self.task.index+1} done")

    def complete_step(self) -> bool:
        """Advance after 'stop'. Returns True when the whole task is finished."""
        self.task.index += 1
        self.turns_here = 0
        return self.task.current is None


def view_mosaic(views, width=320):
    """2x2 sheet of looked-around views, labelled as the VLM saw them (for the decision record)."""
    from PIL import Image, ImageDraw
    tiles = []
    for k, (rgb, yaw) in enumerate(views):
        tile = Image.fromarray(rgb).convert("RGB")
        tile = tile.resize((width, round(tile.height*width/tile.width)))
        ImageDraw.Draw(tile).text((8, 6), f"view {k+1} {compass_name(yaw)}", fill=(255, 255, 0))
        tiles.append(tile)
    h = tiles[0].height
    sheet = Image.new("RGB", (2*width, 2*h))
    for k, tile in enumerate(tiles[:4]):
        sheet.paste(tile, ((k % 2)*width, (k//2)*h))
    return np.asarray(sheet)


def approach_pose(target_xy, near_xy, robot_xy, standoff=1.4, occupied=None):
    """Stand-off pose in front of a visually localized object, facing it.

    The edge to keep `standoff` from is the object's outline on the occupancy map when the localized
    point lies on a mapped obstacle (`occupied(xy)`): tables, conveyors and stations are seen from above,
    so the nearest depth on them can lie well inside their outline. Otherwise (a floor bay, an unmapped
    object) it is the nearest depth on the object, `near_xy`.
    """
    target = np.asarray(target_xy, float)
    robot = np.asarray(robot_xy, float)
    near = np.asarray(near_xy, float)
    bearing = math.atan2(target[1]-robot[1], target[0]-robot[0])
    if occupied is not None and occupied(target):
        span = float(np.linalg.norm(robot-target))
        toward = (robot-target)/max(span, 1e-6)
        for s in np.arange(0.0, span, 0.05):
            if not occupied(target+s*toward):
                near = target+s*toward
                break
    near_dist = float(np.linalg.norm(near-robot))
    step = max(0.0, near_dist-standoff)
    return (float(robot[0]+step*math.cos(bearing)), float(robot[1]+step*math.sin(bearing)), bearing)


def behind_partition(target_xy, robot_xy, occupied, max_wall=0.35, open_behind=0.6, step=0.05) -> bool:
    """The localized point sits on a thin mapped wall with open floor behind it.

    Depth stops at glass partitions and windows, so an object seen through one is localized on the wall
    (measured: the meeting table seen from the corridor). Wall-mounted objects (cabinets, panels) have the
    building wall or the map edge behind them, free-standing ones are thicker than a partition.
    """
    target, robot = np.asarray(target_xy, float), np.asarray(robot_xy, float)
    if not occupied(target):
        return False
    span = float(np.linalg.norm(target-robot))
    away = (target-robot)/max(span, 1e-6)
    s = 0.0
    while s < span and occupied(target-s*away):
        s += step
    near = target-s*away                       # first free point on the robot's side
    depth = step
    while depth < 3.0 and occupied(near+depth*away):
        depth += step
    if depth-step > max_wall:
        return False
    return all(not occupied(near+(depth+k*step)*away) for k in range(int(open_behind/step)))


def already_in_front(goal, robot_xy, tolerance=0.4) -> bool:
    """An 'approach' whose stand-off pose is where the robot already stands means the step is done."""
    return math.hypot(goal[0]-robot_xy[0], goal[1]-robot_xy[1]) < tolerance
