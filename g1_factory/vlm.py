"""VLM layer of the hierarchical VLN stack (OpenAI-compatible chat API).

The same client talks to Ollama on a laptop (``http://localhost:11434/v1``),
vLLM/SGLang on a GPU server, or any OpenAI-compatible endpoint. Three calls:

* ``plan_task``  - instruction + semantic map -> ordered place ids + what to check
* ``locate``     - camera image + target phrase -> bounding box (visual grounding)
* ``inspect``    - camera image + question -> short answer for the external agent

Model output is validated: place ids must exist in the semantic map, boxes must
lie inside the image. Invalid output is retried once, then the caller falls back
(rule-based grounding for planning; "not visible" for grounding).
"""
from __future__ import annotations

import base64
from dataclasses import dataclass
from io import BytesIO
import json
import re
import time
from urllib.request import Request, urlopen

import numpy as np

from .grounding import GroundedTask, SemanticMap, Subgoal, ground_instruction, _subgoal


def encode_jpeg(image, quality: int = 85) -> bytes:
    from PIL import Image
    pil = image if isinstance(image, Image.Image) else Image.fromarray(np.asarray(image, dtype=np.uint8))
    buffer = BytesIO()
    pil.convert("RGB").save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()


def extract_json(text: str) -> dict:
    """First top-level JSON object in a model reply (tolerates ``` fences and prose)."""
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    start = text.find("{")
    while start >= 0:
        depth, in_str, escape = 0, False, False
        for end in range(start, len(text)):
            ch = text[end]
            if in_str:
                if escape:
                    escape = False
                elif ch == "\\":
                    escape = True
                elif ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    try:
                        value = json.loads(text[start:end+1])
                    except json.JSONDecodeError:
                        break
                    if isinstance(value, dict):
                        return value
                    break
        start = text.find("{", start+1)
    raise ValueError(f"No JSON object in model reply: {text[:200]!r}")


@dataclass
class VLMCall:
    text: str
    latency_s: float
    request_bytes: int
    response_bytes: int
    images: int
    prompt_tokens: int | None = None
    completion_tokens: int | None = None


class VLMClient:
    def __init__(self, base_url: str, model: str, timeout: float = 180.0, api_key: str | None = None):
        self.url = base_url.rstrip("/")+"/chat/completions"
        self.model, self.timeout, self.api_key = model, timeout, api_key
        self.calls: list[VLMCall] = []

    def chat(self, system: str, text: str, images=(), max_tokens: int = 400) -> VLMCall:
        content = [{"type": "text", "text": text}]
        for image in images:
            data = image if isinstance(image, (bytes, bytearray)) else encode_jpeg(image)
            content.append({"type": "image_url",
                            "image_url": {"url": "data:image/jpeg;base64,"+base64.b64encode(data).decode("ascii")}})
        body = json.dumps({"model": self.model, "temperature": 0, "max_tokens": max_tokens,
                           "messages": [{"role": "system", "content": system},
                                        {"role": "user", "content": content}]}).encode()
        headers = {"Content-Type": "application/json"}
        if self.api_key:
            headers["Authorization"] = "Bearer "+self.api_key
        begin = time.monotonic()
        with urlopen(Request(self.url, data=body, headers=headers), timeout=self.timeout) as response:
            raw = response.read()
        result = json.loads(raw)
        usage = result.get("usage") or {}
        call = VLMCall(result["choices"][0]["message"]["content"] or "", time.monotonic()-begin, len(body),
                       len(raw), len(images), usage.get("prompt_tokens"), usage.get("completion_tokens"))
        self.calls.append(call)
        return call


PLAN_SYSTEM = """You are the high-level planner (layer 1) of a hierarchical vision-language navigation \
system on a Unitree G1 humanoid robot in a factory campus (a warehouse and an office building). \
An external agent sends an instruction such as "go to <part of the factory> and check <something>". \
Choose where the robot must go, using ONLY place ids from the semantic map, in visiting order. \
If the object to check is (or is part of) a listed landmark, the LAST subgoal must be that landmark id; \
use a zone id only when no landmark fits. \
Also say what the robot must look at when it arrives and the question it must answer from its camera. \
`target` is the fixed object or spot the camera must point at to answer (e.g. "charging dock" for \
"is a cart parked at the dock?", "meeting table" for "is anyone at the table?"), never an object whose \
presence is being asked about. \
Reply with one JSON object and nothing else:
{"subgoals": ["<place_id>", ...], "target": "<short English noun phrase of the object to look at>", \
"question": "<what to check, in the language of the instruction>", "language": "ko|en", "reason": "<one short sentence>"}"""

LOCATE_SYSTEM = """You are the visual grounding module of a robot. Find the requested object in the \
robot's head-camera image. Reply with one JSON object and nothing else:
{"found": true|false, "bbox_2d": [x1, y1, x2, y2]} with coordinates normalized to 0-1000 \
(x to the right, y down). If the object is not visible, reply {"found": false}."""

INSPECT_SYSTEM = """You are the inspection module of a factory robot. Answer the question using only \
what is visible in the robot's head-camera image. First describe what is actually on or at the place \
being asked about (`observed`), then answer. `verdict` answers the question with yes or no, consistent \
with `observed`. Absence is an answer: if the place is clearly in view and the object or person in \
question is not there, the verdict is "no". Use "unknown" only when the place itself is not visible or \
the view is blocked. Reply with one JSON object and nothing else, fields in this order:
{"observed": "<objects on/at the place, in English>", "target_visible": true|false, \
"verdict": "yes|no|unknown", "status": "ok|anomaly|unknown", \
"answer": "<1-2 sentences in the requested language>"}"""


def semantic_map_prompt(smap: SemanticMap) -> str:
    lines = []
    for place in smap.places.values():
        aliases = ", ".join(place.aliases[:4])
        lines.append(f"- {place.id} | {place.label} | {place.kind} in {place.zone} | aliases: {aliases}")
    return "\n".join(lines)


class HierarchicalVLM:
    """Layer-1 reasoning and visual checks; navigation itself stays in the planner/tracker."""

    def __init__(self, client: VLMClient, smap: SemanticMap):
        self.client, self.smap = client, smap
        self._map_text = semantic_map_prompt(smap)

    def plan_task(self, payload: str) -> tuple[GroundedTask, dict]:
        """Structured JSON `subgoals` are honoured as-is; language goes through the VLM."""
        data = None
        if payload.lstrip().startswith("{"):
            data = json.loads(payload)
            if "subgoals" in data:
                task = ground_instruction(payload, self.smap)
                return task, {"source": "external_subgoals", "target": data.get("target", ""),
                              "question": data.get("question", ""), "language": data.get("language", "")}
        instruction = data.get("instruction", "") if data else payload
        task_id = str((data or {}).get("task_id") or "")
        prompt = f"Semantic map (id | label | type | aliases):\n{self._map_text}\n\nInstruction: {instruction}"
        error = None
        for attempt in range(2):
            text = prompt if error is None else prompt+f"\n\nYour previous reply was invalid: {error}. Use only listed ids."
            call = self.client.chat(PLAN_SYSTEM, text, max_tokens=300)
            try:
                reply = extract_json(call.text)
                ids = reply.get("subgoals")
                if not isinstance(ids, list) or not ids or not all(isinstance(i, str) for i in ids):
                    raise ValueError("subgoals must be a non-empty list of place ids")
                unknown = [i for i in ids if i not in self.smap.places]
                if unknown:
                    raise ValueError(f"unknown place ids {unknown}")
                ids = self.refine_to_landmark(ids, f"{reply.get('target', '')} {reply.get('question', '')} {instruction}")
                subgoals = []
                for place_id in ids:
                    if not subgoals or subgoals[-1].place_id != place_id:
                        subgoals.append(_subgoal(self.smap.places[place_id], "vlm"))
                task = GroundedTask(task_id or ground_instruction(instruction, self.smap).task_id,
                                    subgoals, "vlm", instruction)
                return task, {"source": "vlm", "target": str(reply.get("target", "")),
                              "question": str(reply.get("question", "")), "language": str(reply.get("language", "")),
                              "reason": str(reply.get("reason", "")), "attempts": attempt+1,
                              "latency_s": call.latency_s, "request_bytes": call.request_bytes,
                              "response_bytes": call.response_bytes, "raw": call.text}
            except ValueError as exc:
                error = str(exc)
        # Deterministic fallback keeps the robot usable when the VLM output is unusable.
        task = ground_instruction(payload, self.smap)
        task.parser = "rule_fallback"
        return task, {"source": "rule_fallback", "error": error, "target": "", "question": instruction, "language": ""}

    def refine_to_landmark(self, ids: list, text: str) -> list:
        """Small VLMs often stop at the zone; append the zone's landmark named in the check text."""
        last = self.smap.places[ids[-1]]
        if last.kind != "zone":
            return ids
        for mention in self.smap.find_mentions(text):
            place = self.smap.places[mention["place"]]
            if place.kind == "landmark" and place.zone == last.id and not mention["generic"]:
                return [*ids, place.id]
        return ids

    def locate(self, image: np.ndarray, target: str):
        """Return (x1, y1, x2, y2) in pixels, or None when not visible."""
        h, w = image.shape[:2]
        call = self.client.chat(LOCATE_SYSTEM, f"Object to find: {target}", [image], max_tokens=120)
        try:
            reply = extract_json(call.text)
        except ValueError:
            return None, call
        box = reply.get("bbox_2d")
        if not reply.get("found") or not isinstance(box, list) or len(box) != 4:
            return None, call
        try:
            x1, y1, x2, y2 = (float(v) for v in box)
        except (TypeError, ValueError):
            return None, call
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        if not (0 <= x1 < x2 <= 1000 and 0 <= y1 < y2 <= 1000):
            return None, call
        return (x1*w/1000, y1*h/1000, x2*w/1000, y2*h/1000), call

    def inspect(self, image: np.ndarray, instruction: str, question: str, language: str,
                place: str = "") -> tuple[dict, VLMCall]:
        lang = "Korean" if language.lower().startswith("ko") or re.search(r"[가-힣]", instruction) else "English"
        where = f"The robot is at: {place}; the camera faces it.\n" if place else ""
        text = f"{where}Instruction from the operator: {instruction}\nQuestion: {question or instruction}\nAnswer in {lang}."
        call = self.client.chat(INSPECT_SYSTEM, text, [image], max_tokens=300)
        try:
            reply = extract_json(call.text)
        except ValueError:
            reply = {"target_visible": None, "status": "unknown", "answer": call.text.strip()[:500]}
        verdict = str(reply.get("verdict", "unknown")).strip().lower()
        return {"observed": str(reply.get("observed", "")), "target_visible": reply.get("target_visible"),
                "verdict": verdict if verdict in ("yes", "no") else "unknown",
                "status": str(reply.get("status", "unknown")), "answer": str(reply.get("answer", ""))}, call
