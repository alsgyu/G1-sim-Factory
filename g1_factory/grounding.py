"""Hierarchical VLN layer 1: instruction from an external agent -> ordered map goals.

Unlike end-to-end generative VLN (NaVILA: frames + text -> "move forward 75 cm"),
this layer only decides WHERE to go. It grounds an instruction against a
semantic map of named places and hands map-frame goal poses to the planner
(layer 2), which hands paths to the tracker (layer 3) and the G1 policy (layer 4).

Accepted instruction payloads (std_msgs/String on /vln/instruction):
  * Structured JSON - preferred for agent-to-robot messages:
      {"task_id": "t1", "subgoals": ["inspection", {"place": "charging_dock_3"},
                                      {"x": 12.0, "y": -3.8, "yaw": 1.57, "label": "custom"}]}
  * JSON with natural language:  {"task_id": "t1", "instruction": "Go to ..."}
  * Plain natural-language text (English or Korean).
Natural language is grounded by alias matching in order of mention; it does not
parse negation or "before/after" reordering. Agents needing those should send
explicit `subgoals`.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import difflib
import json
import math
from pathlib import Path
import re
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEMANTIC_MAP = ROOT/"configs/semantic_map.yaml"


def _compact(text: str) -> str:
    return re.sub(r"[\s_\-]+", "", text.lower())


@dataclass(frozen=True)
class Place:
    id: str
    label: str
    kind: str            # zone | landmark
    zone: str
    pose: tuple          # (x, y, yaw) world/map frame approach pose
    aliases: tuple
    generic_aliases: tuple = ()
    contains_zones: tuple = ()
    annotated: bool = True      # on the robot's annotated SLAM map (False: evaluation only)
    object_xy: tuple | None = None
    footprints: tuple = ()      # ((x1, y1, x2, y2), ...) floor outline(s) of the object, evaluation only

    def distance_to_object(self, x: float, y: float) -> float:
        """Metres from (x, y) to the nearest footprint box, else to the object point, else to the pose."""
        if self.footprints:
            return min(math.hypot(max(x1-x, 0.0, x-x2), max(y1-y, 0.0, y-y2)) for x1, y1, x2, y2 in self.footprints)
        ox, oy = self.object_xy or self.pose[:2]
        return math.hypot(x-ox, y-oy)


@dataclass
class Subgoal:
    place_id: str
    label: str
    x: float
    y: float
    yaw: float | None
    matched: str = ""


@dataclass
class GroundedTask:
    task_id: str
    subgoals: list
    parser: str                      # json_subgoals | language
    instruction: str = ""
    unresolved: list = field(default_factory=list)
    suggestions: list = field(default_factory=list)

    @property
    def ok(self):
        return bool(self.subgoals) and not self.unresolved

    def to_dict(self):
        return {**asdict(self), "ok": self.ok}


class GroundingError(ValueError):
    pass


class SemanticMap:
    def __init__(self, places: dict):
        self.places = places
        self._alias_index = []  # (compact alias, place id, original alias, generic)
        for place in places.values():
            names = {place.id, place.id.replace("_", " "), place.label, *place.aliases}
            for name in names:
                key = _compact(name)
                if key:
                    self._alias_index.append((key, place.id, name, name in place.generic_aliases))
        self._alias_index.sort(key=lambda item: -len(item[0]))

    @classmethod
    def load(cls, factory_config: dict, path: str | Path = DEFAULT_SEMANTIC_MAP):
        spec = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        aliases = spec.get("zone_aliases", {})
        places = {}
        for zone in factory_config["zones"]:
            gx, gy = zone["goal"]
            cx, cy = zone["center"]
            places[zone["id"]] = Place(zone["id"], zone["label"], "zone", zone["id"],
                                       (float(gx), float(gy), math.atan2(cy-gy, cx-gx)),
                                       tuple(aliases.get(zone["id"], ())))
        for item in spec.get("landmarks", []):
            if item["id"] in places:
                raise ValueError(f"Duplicate semantic place id: {item['id']}")
            pose = tuple(float(v) for v in item["pose"])
            if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f"{item['id']}.pose must be finite [x, y, yaw]")
            places[item["id"]] = Place(item["id"], item["label"], "landmark", item.get("zone", "transit"), pose,
                                       tuple(item.get("aliases", ())), tuple(item.get("generic_aliases", ())),
                                       tuple(item.get("contains_zones", ())), bool(item.get("annotated", False)),
                                       tuple(float(v) for v in item["object"]) if "object" in item else None,
                                       tuple(tuple(float(v) for v in box) for box in item.get("footprint", ())))
            if any(len(b) != 4 or b[0] >= b[2] or b[1] >= b[3] for b in places[item["id"]].footprints):
                raise ValueError(f"{item['id']}.footprint boxes must be [x1, y1, x2, y2] with x1 < x2, y1 < y2")
        unknown = {z for p in places.values() for z in p.contains_zones} - set(places)
        if unknown:
            raise ValueError(f"contains_zones references unknown zones: {sorted(unknown)}")
        return cls(places)

    def annotated(self) -> "SemanticMap":
        """Only what a person annotated on the SLAM map: zones and infrastructure anchors."""
        return SemanticMap({k: p for k, p in self.places.items() if p.annotated})

    def resolve(self, name: str) -> Place:
        if name in self.places:
            return self.places[name]
        key = _compact(name)
        for alias, place_id, _, _ in self._alias_index:
            if alias == key:
                return self.places[place_id]
        raise GroundingError(f"Unknown place {name!r}")

    def suggest(self, text: str, n: int = 3):
        words = re.findall(r"[\w가-힣]+", text.lower())
        grams = {" ".join(words[i:i+k]) for k in (1, 2, 3) for i in range(len(words)-k+1)}
        names = sorted({alias for _, _, alias, _ in self._alias_index})
        found = []
        for gram in sorted(grams, key=len, reverse=True):
            for match in difflib.get_close_matches(gram, names, n=1, cutoff=0.75):
                place = self.resolve(match).id
                if place not in found:
                    found.append(place)
        return found[:n]

    def find_mentions(self, text: str):
        """Non-overlapping alias matches, longest alias first, in order of mention."""
        compact = []
        positions = []
        for index, char in enumerate(text.lower()):
            if not re.match(r"[\s_\-]", char):
                compact.append(char)
                positions.append(index)
        haystack = "".join(compact)
        taken = [False]*len(haystack)
        mentions = []
        for alias, place_id, original, generic in self._alias_index:
            start = haystack.find(alias)
            while start >= 0:
                end = start+len(alias)
                # A numbered alias must not match a longer number ("dock 1" vs "dock 12").
                after = haystack[end] if end < len(haystack) else ""
                boundary_ok = not (after.isdigit() and alias[-1].isdigit())
                if boundary_ok and not any(taken[start:end]):
                    for k in range(start, end):
                        taken[k] = True
                    mentions.append({"start": positions[start], "place": place_id, "alias": original, "generic": generic})
                start = haystack.find(alias, start+1)
        mentions.sort(key=lambda m: m["start"])
        return mentions


def _subgoal(place: Place, matched: str = "") -> Subgoal:
    x, y, yaw = place.pose
    return Subgoal(place.id, place.label, x, y, yaw, matched)


def ground_language(text: str, smap: SemanticMap):
    mentions = smap.find_mentions(text)
    # A generic container word ("office") is dropped when a place inside it is named.
    named_zones = {smap.places[m["place"]].zone for m in mentions}
    mentions = [m for m in mentions
                if not (m["generic"] and set(smap.places[m["place"]].contains_zones) & named_zones)]
    # "the conveyor in the assembly area": a zone right after one of its own
    # landmarks is a locative phrase, not another destination.
    kept = []
    for m in mentions:
        place = smap.places[m["place"]]
        if kept and place.kind == "zone" and smap.places[kept[-1]["place"]].zone == place.id \
                and smap.places[kept[-1]["place"]].kind == "landmark":
            continue
        if kept and kept[-1]["place"] == m["place"]:
            continue
        kept.append(m)
    return [_subgoal(smap.places[m["place"]], m["alias"]) for m in kept]


def ground_instruction(payload: str, smap: SemanticMap) -> GroundedTask:
    """Parse one /vln/instruction payload. Never invents a destination."""
    if not isinstance(payload, str) or not payload.strip():
        raise GroundingError("Instruction must be non-empty text")
    if len(payload) > 8000:
        raise GroundingError("Instruction must be at most 8000 characters")
    data = None
    if payload.lstrip().startswith("{"):
        try:
            data = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise GroundingError(f"Invalid JSON instruction: {exc}") from exc
        if not isinstance(data, dict):
            raise GroundingError("JSON instruction must be an object")
    task_id = str((data or {}).get("task_id") or uuid.uuid4().hex[:8])
    if data is not None and "subgoals" in data:
        items = data["subgoals"]
        if not isinstance(items, list) or not items or len(items) > 32:
            raise GroundingError("subgoals must be a list of 1-32 places or poses")
        subgoals, unresolved = [], []
        for item in items:
            if isinstance(item, str) or (isinstance(item, dict) and "place" in item):
                name = item if isinstance(item, str) else item["place"]
                try:
                    subgoals.append(_subgoal(smap.resolve(str(name)), str(name)))
                except GroundingError:
                    unresolved.append(str(name))
            elif isinstance(item, dict) and {"x", "y"} <= set(item):
                try:
                    x, y = float(item["x"]), float(item["y"])
                    yaw = None if item.get("yaw") is None else float(item["yaw"])
                except (TypeError, ValueError) as exc:
                    raise GroundingError(f"Pose subgoal values must be numbers: {item}") from exc
                if not all(math.isfinite(v) for v in (x, y, 0.0 if yaw is None else yaw)):
                    raise GroundingError("Pose subgoal must be finite")
                subgoals.append(Subgoal("pose", str(item.get("label", "pose")), x, y, yaw, "pose"))
            else:
                raise GroundingError(f"Unsupported subgoal entry: {item!r}")
        return GroundedTask(task_id, subgoals, "json_subgoals", str(data.get("instruction", "")), unresolved,
                            [s for name in unresolved for s in smap.suggest(name)])
    text = data.get("instruction", "") if data is not None else payload
    if not isinstance(text, str) or not text.strip():
        raise GroundingError("JSON instruction needs `subgoals` or a non-empty `instruction`")
    subgoals = ground_language(text, smap)
    if not subgoals:
        return GroundedTask(task_id, [], "language", text, [text], smap.suggest(text))
    return GroundedTask(task_id, subgoals, "language", text)
