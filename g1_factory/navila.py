"""Optional NaVILA HTTP bridge. The fixed-route baseline does not import a VLM."""

from __future__ import annotations

import base64
from dataclasses import dataclass
from io import BytesIO
import json
import math
import re
from urllib.request import Request, urlopen


@dataclass(frozen=True)
class NavAction:
    """Incremental action: metres for forward; degrees for left/right turns."""

    kind: str
    value: float = 0.0


def parse_navila_action(text: str) -> NavAction:
    """Accept exactly one native action; cap execution to 0.5 m or 30 degrees.

    Explanations, multiple actions, negative/non-finite values, and unsupported
    actions raise ValueError. The caller must stop the episode on parse failure.
    """
    if not isinstance(text, str):
        raise ValueError("NaVILA response must be text")
    normalized = text.strip().lower().rstrip(".").strip()
    if normalized == "stop":
        return NavAction("stop")
    match = re.fullmatch(
        r"move forward\s+(\d+(?:\.\d+)?)\s*(cm|centimeters?|m|meters?)",
        normalized,
    )
    if match:
        distance = float(match[1]) / (100.0 if match[2].startswith("c") else 1.0)
        if not math.isfinite(distance) or not 0 < distance <= 10:
            raise ValueError("Forward distance must be in (0, 10] metres")
        return NavAction("move_forward", min(distance, 0.5))
    match = re.fullmatch(
        r"turn\s+(left|right)\s+(\d+(?:\.\d+)?)\s*(?:degrees?|deg|°)",
        normalized,
    )
    if match:
        angle = float(match[2])
        if not math.isfinite(angle) or not 0 < angle <= 180:
            raise ValueError("Turn angle must be in (0, 180] degrees")
        return NavAction("turn_" + match[1], min(angle, 30.0))
    raise ValueError(f"Unsupported NaVILA action: {text!r}")


class NaVILAClient:
    """Send RGB observations to a separately installed, actual pretrained model."""

    def __init__(self, url: str, timeout: float = 120.0):
        self.url = url.rstrip("/") + "/infer"
        self.timeout = timeout
        self.last_text = ""

    def infer(self, instruction: str, frames: list) -> NavAction:
        from PIL import Image

        if not instruction.strip() or not frames:
            raise ValueError("An instruction and at least one RGB frame are required")
        encoded = []
        for frame in frames[-8:]:
            image = frame if isinstance(frame, Image.Image) else Image.fromarray(frame)
            image = image.convert("RGB")
            image.thumbnail((640, 480))
            buffer = BytesIO()
            image.save(buffer, format="JPEG", quality=90)
            encoded.append(base64.b64encode(buffer.getvalue()).decode("ascii"))
        payload = json.dumps({"instruction": instruction, "frames": encoded}).encode()
        request = Request(self.url, data=payload, headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=self.timeout) as response:
            result = json.load(response)
        if not isinstance(result, dict) or not isinstance(result.get("text"), str):
            raise ValueError("NaVILA server must return a JSON object with text")
        self.last_text = result["text"]
        # Parse the original text locally instead of trusting server-provided actions.
        return parse_navila_action(self.last_text)
