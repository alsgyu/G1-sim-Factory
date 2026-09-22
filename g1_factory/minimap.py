"""Lightweight, world-coordinate site map for the viewer's debug overlay.

This schematic uses scene metadata, never a robot camera or an extra MuJoCo
renderer. Its RGB output belongs only in the viewer, not in VLN observations.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


_ZONE_CODES = {
    "material_storage": "M", "assembly_line": "A",
    "inspection": "I", "charging": "C",
    "office_workstations": "W", "office_meeting": "M",
    "office_lounge": "L", "office_control": "C",
}


def _font(size: int, bold: bool = False):
    suffix = "-Bold" if bold else ""
    for filename in (
        f"/usr/share/fonts/truetype/dejavu/DejaVuSans{suffix}.ttf",
        f"/usr/share/fonts/truetype/liberation2/LiberationSans{'-Bold' if bold else '-Regular'}.ttf",
    ):
        if Path(filename).is_file():
            return ImageFont.truetype(filename, size)
    return ImageFont.load_default()


class MiniMap:
    """Cache the floor plan and paint only route, target and live G1 pose.

    ``metadata`` is the dictionary returned by ``scene.build_scene``. Edits to
    obstacle bounds (including ``scene.move_shelf``) invalidate the background
    automatically; ``refresh`` also supports explicit invalidation.
    """

    def __init__(self, metadata: dict, width: int = 360, height: int = 230):
        self.metadata = metadata
        self.width, self.height = int(width), int(height)
        if self.width < 240 or self.height < 180:
            raise ValueError("MiniMap needs at least 240 x 180 pixels")
        scale = min(self.width / 360, self.height / 230)
        self._small = _font(max(8, round(10 * scale)))
        self._title = _font(max(10, round(12 * scale)), bold=True)
        self._badge = _font(max(9, round(11 * scale)), bold=True)
        self._signature = None
        self._background = None
        self.refresh()

    def _fingerprint(self):
        return (
            tuple(self.metadata["bounds"]),
            tuple(self.metadata["warehouse_bounds"]),
            tuple(self.metadata["office_bounds"]),
            float(self.metadata.get("config", {}).get("layout", {}).get("corridor_width", 4)),
            tuple((name, zone.get("kind"), tuple(zone["bounds"]))
                  for name, zone in self.metadata["zones"].items()),
            tuple((obstacle.get("id"), tuple(obstacle["bounds"]))
                  for obstacle in self.metadata.get("obstacles", [])),
        )

    def refresh(self) -> None:
        """Rebuild the static layer from current scene metadata."""
        self.bounds = tuple(float(v) for v in self.metadata["bounds"])
        x0, y0, x1, y1 = self.bounds
        if not all(math.isfinite(v) for v in self.bounds) or x1 <= x0 or y1 <= y0:
            raise ValueError("MiniMap bounds must be finite xmin,ymin,xmax,ymax")
        # Preserve aspect ratio and reserve separate header and legend bands.
        left, top, right, bottom = 12.0, 36.0, self.width - 12.0, self.height - 44.0
        self.scale = min((right - left) / (x1 - x0), (bottom - top) / (y1 - y0))
        self._left = left + ((right - left) - (x1 - x0) * self.scale) / 2
        self._top = top + ((bottom - top) - (y1 - y0) * self.scale) / 2

        background = Image.new("RGB", (self.width, self.height), (16, 24, 33))
        draw = ImageDraw.Draw(background)
        draw.rounded_rectangle((0, 0, self.width - 1, self.height - 1), radius=10,
                               outline=(81, 103, 119), width=1)
        draw.text((12, 7), "SITE MAP", font=self._title, fill=(234, 242, 247))
        for key, label, color in (
            ("warehouse_bounds", "WAREHOUSE", (24, 44, 51)),
            ("office_bounds", "OFFICE", (35, 44, 62)),
        ):
            bounds = self.metadata[key]
            draw.rectangle(self._pixel_box(bounds), fill=color, outline=(101, 123, 137))
            center_x = self.world_to_pixel(((bounds[0] + bounds[2]) / 2, 0))[0]
            draw.text((center_x, 24), label, anchor="mm", font=self._small, fill=(172, 192, 205))
        warehouse, office = self.metadata["warehouse_bounds"], self.metadata["office_bounds"]
        half_corridor = self.metadata.get("config", {}).get("layout", {}).get("corridor_width", 4) / 2
        draw.rectangle(self._pixel_box((warehouse[2], -half_corridor, office[0], half_corridor)),
                       fill=(47, 63, 73), outline=(101, 123, 137))
        for zone in self.metadata["zones"].values():
            color = (35, 65, 67) if zone.get("kind") == "warehouse" else (46, 60, 83)
            draw.rectangle(self._pixel_box(zone["bounds"]), fill=color, outline=(73, 99, 111))
        for obstacle in self.metadata.get("obstacles", []):
            draw.rectangle(self._pixel_box(obstacle["bounds"]), fill=(121, 136, 144))
        for name, zone in self.metadata["zones"].items():
            x0, y0, x1, y1 = zone["bounds"]
            center = self.world_to_pixel(((x0 + x1) / 2, (y0 + y1) / 2))
            x, y = center
            draw.rounded_rectangle((x - 7, y - 8, x + 7, y + 8), radius=3,
                                   fill=(19, 31, 41), outline=(128, 155, 169))
            draw.text(center, _ZONE_CODES.get(name, name[0].upper()), font=self._badge,
                      anchor="mm", fill=(234, 242, 247))
        # The world Y axis points up; this compass never changes with the camera.
        draw.text((self.width - 13, 24), "+Y", anchor="rm", font=self._small, fill=(172, 192, 205))
        footer = self.height - 34
        draw.line((12, footer - 5, self.width - 12, footer - 5), fill=(54, 71, 83))
        self._fit_text(draw, (12, footer), "WH  M Materials   A Assembly   I Inspect   C Charge")
        self._fit_text(draw, (12, footer + 15), "OF   W Work   M Meeting   L Lounge   C Control")
        self._background = background
        self._signature = self._fingerprint()

    def _fit_text(self, draw, position, text):
        font = self._small
        while draw.textlength(text, font=font) > self.width - 24 and getattr(font, "size", 8) > 7:
            font = _font(font.size - 1)
        draw.text(position, text, font=font, fill=(173, 193, 207))

    def world_to_pixel(self, xy) -> tuple[float, float]:
        """Map world XY metres to image pixels, with positive Y toward the top."""
        x, y = float(xy[0]), float(xy[1])
        return self._left + (x - self.bounds[0]) * self.scale, self._top + (self.bounds[3] - y) * self.scale

    def _pixel_box(self, bounds):
        x0, y0, x1, y1 = bounds
        return (*self.world_to_pixel((x0, y1)), *self.world_to_pixel((x1, y0)))

    def render(self, xy, yaw: float, path=None, next_target=None) -> np.ndarray:
        """Return a contiguous uint8 RGB frame, suitable for viewer.set_images."""
        if self._signature != self._fingerprint():
            self.refresh()
        x, y = float(xy[0]), float(xy[1])
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("MiniMap pose must be finite")
        frame = self._background.copy()
        draw = ImageDraw.Draw(frame)
        if path is not None:
            points = np.asarray(path, dtype=float)
            if points.size and (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()):
                raise ValueError("MiniMap path must contain finite [x,y] points")
            if len(points) > 1:
                draw.line([self.world_to_pixel(point) for point in points], fill=(84, 205, 235), width=2)
        if next_target is not None:
            tx, ty = self.world_to_pixel(next_target)
            if not math.isfinite(tx) or not math.isfinite(ty):
                raise ValueError("MiniMap target must be finite")
            draw.polygon([(tx, ty-5), (tx+5, ty), (tx, ty+5), (tx-5, ty)],
                         fill=(91, 218, 248), outline=(226, 250, 255))
        px, py = self.world_to_pixel((x, y))
        # World yaw is counterclockwise; image Y is inverted.
        forward = np.array((math.cos(yaw), -math.sin(yaw)))
        lateral = np.array((-forward[1], forward[0]))
        center = np.array((px, py))
        vertices = [tuple(center + forward * 10),
                    tuple(center - forward * 6 + lateral * 5),
                    tuple(center - forward * 3),
                    tuple(center - forward * 6 - lateral * 5)]
        draw.ellipse((px-10, py-10, px+10, py+10), fill=(16, 24, 33), outline=(248, 178, 83))
        draw.polygon(vertices, fill=(255, 183, 77))
        draw.line([*vertices, vertices[0]], fill=(255, 244, 213), width=1)
        draw.text((self.width - 12, 9), f"G1  x {x:.1f}  y {y:.1f} m", anchor="ra",
                  font=self._small, fill=(255, 193, 104))
        return np.ascontiguousarray(frame, dtype=np.uint8)
