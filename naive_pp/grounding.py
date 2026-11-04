"""Instruction -> a place of the robot's map, by rules (no language model).

The places are what the person annotated on the robot's SLAM map: rooms and corridors with a label and aliases,
and a few anchors. Each becomes a set of phrases (label, id, aliases, and the label with its head noun swapped:
"Charging station" -> "charging area", "Operations control" -> "operations control room"). An instruction is
matched against them in three tiers, and the strongest tier present decides:

  3  exact   a phrase appears as words (English) or as a substring with or without spaces (Korean: particles attach)
  2  partial the content words of a phrase appear together ("the assembly area" -> Assembly line)
  1  keyword optional things a person listed for a room (configs/place_keywords.yaml: "server rack" ->
             Operations control), used only when no place is named at all

Several places in one instruction get a role from the words around them: start ("from X", "X에서 출발"),
via ("through X", "X를 거쳐"), destination ("to X", "X로", "X에 가서") or locative ("in X", "X의"). The destination is
the last place with a destination word, else the last unmarked one, else the first locative one; via places and
earlier destinations become waypoints. Words that only describe what to check ("whether the status indicator lights
on the server rack are on") never move the target away from a named place.

Payloads (std_msgs/String on /vln/instruction, the topic the external agent already uses):
  "Go to the operations control room and check whether ..."          plain English or Korean text
  {"task_id": "t1", "instruction": "..."}                           JSON with text
  {"task_id": "t1", "place": "office_control"}                      a place or anchor id, label or alias
  {"task_id": "t1", "goal": {"x": 58.6, "y": -8.7, "yaw": -1.57}}   a pose in the map package's frame
  {"task_id": "t1", "subgoals": ["bridge", "office_lounge"]}         older agents: waypoints, last = destination
  {"cancel": true}, "stop", "취소"                                    stop and hold
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

GENERIC_EN = {"room", "area", "zone", "station", "section", "space", "corridor", "hallway", "hall", "aisle", "passage",
              "place", "line", "bay"}
HEAD_NOUNS = ("room", "area", "zone", "station", "section", "space")
GENERIC_KO = {"구역", "통로", "복도", "방", "실", "공간", "지역", "존", "쪽"}
ARTICLES = {"the", "a", "an", "this", "that", "our", "your", "its"}
MOTION = {"go", "head", "walk", "move", "navigate", "proceed", "come", "return", "drive", "run", "travel", "get",
          "enter", "reach", "visit", "arrive", "approach", "back"}
EN_START = {"from", "leave", "leaving", "out of", "starting at", "starting from", "start at", "start from", "exit",
            "exiting"}
EN_VIA = {"via", "through", "thru", "past", "passing", "pass", "by way of", "across", "cross", "crossing", "along",
          "passing through", "pass through", "go through", "going through"}
EN_DEST = {"to", "into", "towards", "toward", "reach", "enter", "visit", "until", "up to", "over to", "arrive at",
           "get to", "head to", "go to", "inside"}
EN_LOC = {"in", "inside", "within", "of", "on", "at", "near", "beside", "by", "next to", "around", "behind"}
CANCEL = {"stop", "cancel", "abort", "halt", "freeze", "멈춰", "정지", "취소", "그만", "멈춰라", "중지"}


def normalize(text: str) -> str:
    t = text.lower()
    t = re.sub(r"[_\-/]", " ", t)
    t = re.sub(r"[^0-9a-z가-힣\s]", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def stem(word: str) -> str:
    if len(word) > 4 and word.endswith("ies"):
        return word[:-3]+"y"
    if len(word) > 4 and word.endswith(("ches", "shes", "sses", "xes")):
        return word[:-2]
    if len(word) > 3 and word.endswith("s") and not word.endswith("ss"):
        return word[:-1]
    return word


def hangul(text: str) -> bool:
    return bool(re.search(r"[가-힣]", text))


@dataclass(frozen=True)
class Entry:
    kind: str       # place | anchor
    id: str
    phrase: str     # normalized
    tier: int       # 3 name, 1 keyword
    source: str     # label | alias | id | variant | keyword


@dataclass
class Target:
    kind: str                   # place | anchor | pose
    id: str
    label: str
    place: str | None = None    # place the target is in (anchors, poses)
    xy: tuple | None = None     # anchors and poses
    yaw: float | None = None    # poses only
    matched: str = ""           # instruction words it was grounded from
    match: str = ""             # exact | partial | keyword | id | pose


@dataclass
class Grounding:
    task_id: str
    instruction: str
    target: Target | None = None
    waypoints: list = field(default_factory=list)
    mentions: list = field(default_factory=list)
    alternatives: list = field(default_factory=list)
    cancel: bool = False
    error: str | None = None

    @property
    def ok(self):
        return self.target is not None and self.error is None

    def to_dict(self):
        return asdict(self)


class GroundingError(ValueError):
    pass


class Lexicon:
    """Phrases of every annotated place and anchor of a semantic map (semantic_map.json)."""

    def __init__(self, semantic: dict, keywords: dict | None = None):
        self.places = {p["id"]: p for p in semantic["places"] if p["kind"] in ("zone", "aisle")}
        self.anchors = {a["id"]: a for a in semantic.get("anchors", [])}
        entries = set()
        for p in self.places.values():
            entries |= self._names("place", p["id"], p["label"], p.get("aliases", []))
        for a in self.anchors.values():
            entries |= self._names("anchor", a["id"], a["label"], a.get("aliases", []))
        for pid, words in (keywords or {}).items():
            if pid in self.places:
                entries |= {Entry("place", pid, normalize(w), 1, "keyword") for w in words if normalize(w)}
        # A phrase that names two different targets (an anchor called like its room) belongs to the more specific
        # anchor; a keyword that is also a name is a name.
        by_phrase = {}
        for e in sorted(entries, key=lambda e: (-e.tier, e.kind != "anchor", e.source != "label", e.id, e.source)):
            by_phrase.setdefault(e.phrase, e)
        self.entries = sorted(by_phrase.values(), key=lambda e: -len(e.phrase))

    @staticmethod
    def _names(kind, tid, label, aliases):
        out = {Entry(kind, tid, normalize(label), 3, "label"), Entry(kind, tid, normalize(tid), 3, "id")}
        out |= {Entry(kind, tid, normalize(a), 3, "alias") for a in aliases if normalize(a)}
        for phrase in [label, *aliases]:
            words = normalize(phrase).split()
            if not words or hangul(phrase):
                continue
            stem_words = words[:-1] if words[-1] in GENERIC_EN and len(words) > 1 else words
            for head in HEAD_NOUNS:  # "Charging station" -> "charging area"; "Operations control" -> "... room"
                out.add(Entry(kind, tid, " ".join([*stem_words, head]), 3, "variant"))
        return {e for e in out if e.phrase}

    def label(self, kind, tid):
        return (self.places if kind == "place" else self.anchors)[tid]["label"]

    def resolve(self, name: str) -> Entry | None:
        """A place or anchor named exactly (id, label, alias or variant)."""
        raw = str(name).strip()
        for kind, table in (("place", self.places), ("anchor", self.anchors)):
            if raw in table:
                return Entry(kind, raw, normalize(raw), 3, "id")
        key = normalize(raw)
        return next((e for e in self.entries if e.phrase == key and e.tier == 3), None)

    # ------------------------------------------------------------------ matching
    def mentions(self, text: str) -> list:
        """Non-overlapping matches of the instruction, strongest first per span, in order of mention."""
        norm = normalize(text)
        found = self._exact(norm)+self._partial(norm)
        found.sort(key=lambda m: (-m["tier"], -(m["end"]-m["start"]), -m["score"], m["start"]))
        taken, kept = [], []
        for m in found:
            if any(m["start"] < b and a < m["end"] for a, b in taken):
                continue
            taken.append((m["start"], m["end"]))
            kept.append(m)
        kept.sort(key=lambda m: m["start"])
        for m in kept:
            m["role"] = role(norm, m["start"], m["end"])
        return kept

    def _exact(self, norm: str) -> list:
        out = []
        compact, where = [], []
        for k, ch in enumerate(norm):
            if ch != " ":
                compact.append(ch)
                where.append(k)
        compact = "".join(compact)
        for e in self.entries:
            if hangul(e.phrase):  # Korean particles attach to the noun and spaces are optional: substring match
                key = e.phrase.replace(" ", "")
                start = compact.find(key)
                while start >= 0:
                    out.append(self._mention(e, where[start], where[start+len(key)-1]+1, norm, e.tier, 1.0))
                    start = compact.find(key, start+1)
            else:
                for hit in re.finditer(r"(?<![0-9a-z])"+re.escape(e.phrase)+r"(?:e?s)?(?![0-9a-z])", norm):
                    out.append(self._mention(e, hit.start(), hit.end(), norm, e.tier, 1.0))
        return out

    def _partial(self, norm: str) -> list:
        """Content words of a name that appear close together ("assembly area" for Assembly line)."""
        tokens = [(m.start(), m.end(), m.group()) for m in re.finditer(r"\S+", norm)]
        stems = [stem(t) for _, _, t in tokens]
        out = []
        for e in self.entries:
            if e.tier != 3:
                continue
            words = e.phrase.split()
            if len(words) < 2:
                continue
            ko = hangul(e.phrase)
            generic = GENERIC_KO if ko else GENERIC_EN
            weights = [0.5 if w in generic else 1.0 for w in words]
            hits = {}
            for i, w in enumerate(words):
                for j, (_, _, t) in enumerate(tokens):
                    ok = (t.startswith(w) and len(w) >= 2) if ko else (stem(w) == stems[j])
                    if ok:
                        hits.setdefault(i, j)
            content = [i for i in hits if weights[i] == 1.0 and len(words[i]) >= (2 if ko else 4)]
            if not content:
                continue
            cover = sum(weights[i] for i in hits)/sum(weights)
            pos = sorted(hits[i] for i in hits)
            if cover < 0.5 or pos[-1]-pos[0] > len(words)+1:
                continue
            out.append(self._mention(e, tokens[pos[0]][0], tokens[pos[-1]][1], norm, 2, cover))
        return out

    def _mention(self, e, start, end, norm, tier, score):
        return {"kind": e.kind, "id": e.id, "label": self.label(e.kind, e.id), "phrase": e.phrase,
                "text": norm[start:end], "start": start, "end": end, "tier": tier, "score": round(score, 2),
                "match": {3: "exact", 2: "partial", 1: "keyword"}[tier]}

    def suggest(self, text: str, n: int = 3) -> list:
        words = normalize(text).split()
        grams = {" ".join(words[i:i+k]) for k in (1, 2, 3) for i in range(len(words)-k+1)}
        names = {e.phrase: e for e in self.entries if e.tier == 3}
        found = []
        for gram in sorted(grams, key=len, reverse=True):
            for close in difflib.get_close_matches(gram, list(names), n=1, cutoff=0.75):
                label = self.label(names[close].kind, names[close].id)
                if label not in found:
                    found.append(label)
        return found[:n]


def role(norm: str, start: int, end: int) -> str:
    """start | via | dest | locative | none, from the words just before (English) or just after (Korean) a place."""
    before = [w for w in norm[:start].split()][-4:]
    while before and before[-1] in ARTICLES:
        before.pop()
    grams = {" ".join(before[-k:]) for k in (1, 2, 3) if len(before) >= k}
    after = norm[end:end+12].replace(" ", "")
    if grams & EN_START or after.startswith(("에서출발", "부터", "에서부터", "을떠나", "를떠나")):
        return "start"
    if grams & EN_VIA or re.match(r"^(을|를)?(거쳐|지나|통해|경유|통과)", after):
        return "via"
    if grams & EN_DEST or re.match(r"^(으로|로|까지|에(가|들어|이동|도착|향|가서))", after):
        return "dest"
    if before and before[-1] in ("in", "inside", "at") and MOTION & set(before[:-1]):
        return "dest"  # "go in the pump room", "arrive at ..."
    if grams & EN_LOC or re.match(r"^(의|에있|안의|안에있|내의|쪽의)", after):
        return "locative"
    if after.startswith("에서"):
        return "from_or_at"  # "X에서 Y로" (start) or "X에서 확인해" (the place itself): decided with the others
    return "none"


def pick_destination(mentions: list):
    """(destination mention, waypoint mentions) from the mentions of the strongest tier present."""
    if not mentions:
        return None, []
    tier = max(m["tier"] for m in mentions)
    mentions = [dict(m) for m in mentions]
    later_dest = False
    for m in reversed(mentions):  # "X에서 Y로": X is the start only if a destination follows
        if m["role"] == "from_or_at":
            m["role"] = "start" if later_dest else "dest"
        if m["role"] == "dest":
            later_dest = True
    pool = [m for m in mentions if m["tier"] == tier]
    cands = [m for m in pool if m["role"] not in ("start", "via")]
    marked = [m for m in cands if m["role"] == "dest"]
    plain = [m for m in cands if m["role"] == "none"]
    if marked:
        dest = marked[-1]
    elif plain:
        dest = plain[-1]
    elif cands:
        dest = cands[0]  # only locatives: "the panel on the east wall of the assembly area"
    else:
        rest = [m for m in pool if m["role"] != "start"]
        dest = rest[-1] if rest else pool[-1]
    waypoints = []
    for m in mentions:  # places to pass first: "through the main aisle" counts even when only partly named
        if m["start"] >= dest["start"]:
            break
        if (m["role"] == "via" and m["tier"] >= min(tier, 2)) or (m in pool and m["role"] == "dest"):
            if (m["kind"], m["id"]) != (dest["kind"], dest["id"]) and \
                    (not waypoints or (waypoints[-1]["kind"], waypoints[-1]["id"]) != (m["kind"], m["id"])):
                waypoints.append(m)
    return dest, waypoints


def load_keywords(path) -> dict:
    """{place id: [things in that room]} written by a person (optional)."""
    if not path or not Path(path).is_file():
        return {}
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return {str(k): [str(w) for w in (v or [])] for k, v in data.get("places", data).items()}


class Grounder:
    def __init__(self, semantic: dict, keywords: dict | None = None):
        self.lexicon = Lexicon(semantic, keywords)

    def _target(self, kind, tid, matched="", match="exact") -> Target:
        lx = self.lexicon
        if kind == "anchor":
            a = lx.anchors[tid]
            return Target("anchor", tid, a["label"], a.get("place"), tuple(a["xy"][:2]), None, matched, match)
        return Target("place", tid, lx.places[tid]["label"], tid, None, None, matched, match)

    def ground(self, payload: str) -> Grounding:
        if not isinstance(payload, str) or not payload.strip():
            raise GroundingError("instruction must be non-empty text")
        if len(payload) > 8000:
            raise GroundingError("instruction must be at most 8000 characters")
        data = None
        if payload.lstrip().startswith("{"):
            try:
                data = json.loads(payload)
            except json.JSONDecodeError as exc:
                raise GroundingError(f"invalid JSON instruction: {exc}") from exc
            if not isinstance(data, dict):
                raise GroundingError("JSON instruction must be an object")
        data = data or {}
        task_id = str(data.get("task_id") or uuid.uuid4().hex[:8])
        text = data.get("instruction", data.get("text", "")) if data else payload
        if data.get("cancel") or (isinstance(text, str) and normalize(text) in CANCEL):
            return Grounding(task_id, str(text or ""), cancel=True)
        if not isinstance(text, str):
            raise GroundingError("`instruction` must be text")
        explicit = self._explicit(task_id, text, data)
        if explicit is not None:
            return explicit
        if not text.strip():  # JSON without text: ground what the agent wrote about the steps and the target
            parts = [str(s.get("goal", "")) if isinstance(s, dict) else str(s) for s in data.get("steps", [])]
            text = ". ".join(p for p in [*parts, str(data.get("target", "")), str(data.get("question", ""))] if p)
        return self.ground_text(task_id, text)

    def ground_text(self, task_id: str, text: str) -> Grounding:
        mentions = self.lexicon.mentions(text)
        dest, via = pick_destination(mentions)
        if dest is None:
            known = ", ".join(sorted({p["label"] for p in self.lexicon.places.values()}))
            hint = self.lexicon.suggest(text)
            return Grounding(task_id, text, mentions=mentions, alternatives=hint,
                             error="no place of the map in the instruction"
                                   + (f" (did you mean {', '.join(hint)}?)" if hint else "") + f"; known: {known}")
        if dest["kind"] == "place":  # "the control room ... the server racks": a named anchor inside it is more exact
            inner = [m for m in mentions if m["kind"] == "anchor" and m["tier"] == 3 and m["role"] not in
                     ("start", "via") and self.lexicon.anchors[m["id"]].get("place") == dest["id"]]
            if inner:
                dest = inner[-1]
        others = sorted({m["label"] for m in mentions if m["tier"] == dest["tier"] and m["id"] != dest["id"]
                         and m["role"] not in ("start", "via")})
        return Grounding(task_id, text, self._target(dest["kind"], dest["id"], dest["text"], dest["match"]),
                         [self._target(m["kind"], m["id"], m["text"], m["match"]) for m in via], mentions, others)

    def _explicit(self, task_id, text, data) -> Grounding | None:
        goal = data.get("goal") if isinstance(data.get("goal"), dict) else (data if {"x", "y"} <= set(data) else None)
        if goal is not None:
            return Grounding(task_id, text, self._pose(goal))
        name = data.get("place") or (data.get("target") if isinstance(data.get("target"), dict) else None)
        if isinstance(name, dict):
            name = name.get("place") or name.get("id")
        if name:
            e = self.lexicon.resolve(str(name))
            if e is None:
                raise GroundingError(f"unknown place {name!r}; known: {', '.join(sorted(self.lexicon.places))}")
            return Grounding(task_id, text, self._target(e.kind, e.id, str(name), "id"))
        items = data.get("subgoals")
        if not items:
            return None
        if not isinstance(items, list) or len(items) > 32:
            raise GroundingError("subgoals must be a list of at most 32 places or poses")
        targets = []
        for item in items:
            if isinstance(item, dict) and {"x", "y"} <= set(item):
                targets.append(self._pose(item))
                continue
            name = str(item.get("place") if isinstance(item, dict) else item)
            e = self.lexicon.resolve(name)
            if e is not None:
                targets.append(self._target(e.kind, e.id, name, "id"))
                continue
            g = self.ground_text(task_id, name.replace("_", " "))  # "vending_machine": a thing, not a place
            targets.append(g.target if g.ok else None)
        if not targets or targets[-1] is None:
            return None  # the destination is not on the map: ground the text of the steps and the target instead
        targets = [t for t in targets if t is not None]
        return Grounding(task_id, text, targets[-1], targets[:-1])

    @staticmethod
    def _pose(item) -> Target:
        try:
            x, y = float(item["x"]), float(item["y"])
            yaw = None if item.get("yaw") is None else float(item["yaw"])
        except (TypeError, ValueError, KeyError) as exc:
            raise GroundingError(f"pose goal needs numeric x, y (and optional yaw): {item}") from exc
        if not all(math.isfinite(v) for v in (x, y, 0.0 if yaw is None else yaw)):
            raise GroundingError("pose goal must be finite")
        return Target("pose", "pose", str(item.get("label", "pose")), None, (x, y), yaw, "", "pose")
