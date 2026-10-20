#!/usr/bin/env python3
"""Turn a person's annotations of the robot's SLAM map into the package's semantic map (automatic topology).

    bash scripts/g1_docker.sh python3 scripts/build_map_package.py maps/campus

Reads maps/<name>/map.yaml and annotations.json (tools/map_annotator.html), writes
  semantic_map.json  rooms (names, aliases, bounds, entrance), places and doorways (topology), anchors; map frame
  topology.png       the annotated map with places, doorways and the place graph, to check by eye
  annotation_check.json  simulation only (semantic_truth.json present): rooms, doorways and anchors against the
                     simulator's semantic map
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory import map_package as P  # noqa: E402
from g1_factory import slam_map as S  # noqa: E402

COLORS = [(141, 211, 199), (255, 237, 140), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98),
          (179, 222, 105), (252, 205, 229), (188, 128, 189), (204, 235, 197), (255, 210, 120), (160, 200, 230)]


def font(size, bold=False):
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def topology_figure(grid, origin, res, semantic, scale=3):
    rgb = np.full(grid.shape+(3,), (225, 228, 232), np.uint8)
    rgb[grid == S.FREE] = (255, 255, 255)
    rgb[grid == S.OCCUPIED] = (60, 64, 72)
    img = Image.fromarray(np.flipud(rgb)).resize((grid.shape[1]*scale, grid.shape[0]*scale), Image.NEAREST)
    d = ImageDraw.Draw(img, "RGBA")
    h = grid.shape[0]
    px = lambda x, y: ((x-origin[0])/res*scale, (h-(y-origin[1])/res)*scale)
    color = {}
    for k, p in enumerate(semantic["places"]):
        color[p["id"]] = COLORS[k % len(COLORS)]
        for r in p.get("rects", []) or ([p["bounds"]] if p["kind"] == "area" and p.get("bounds") else []):
            d.rectangle([px(r[0], r[3]), px(r[2], r[1])], fill=(*color[p["id"]], 70), outline=(*color[p["id"]], 255),
                        width=2)
    centers = {p["id"]: p["center"] for p in semantic["places"] if p.get("center")}
    for e in semantic["edges"]:
        if e["a"] in centers and e["b"] in centers:
            d.line([px(*centers[e["a"]]), px(*e["door"]), px(*centers[e["b"]])], fill=(30, 30, 30, 230), width=3)
    for e in semantic["edges"]:
        x, y = px(*e["door"])
        d.ellipse([x-5, y-5, x+5, y+5], fill=(255, 255, 255), outline=(30, 30, 30), width=2)
    for p in semantic["places"]:
        if not p.get("center"):
            continue
        x, y = px(*p["center"])
        r = 9 if p["kind"] == "zone" else 7
        d.ellipse([x-r, y-r, x+r, y+r], fill=color[p["id"]], outline=(30, 30, 30), width=2)
        d.text((x, y-r-3), p["label"], fill=(20, 20, 20), anchor="md", font=font(14, True), stroke_width=3,
               stroke_fill=(255, 255, 255))
    for z in semantic["zones"]:
        if z.get("entrance"):
            x, y = px(*z["entrance"])
            d.rectangle([x-4, y-4, x+4, y+4], fill=(46, 125, 50))
    for a in semantic["anchors"]:
        x, y = px(*a["xy"])
        d.polygon([(x, y-8), (x+7, y+5), (x-7, y+5)], fill=(200, 40, 40))
    return img


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--annotations", type=Path, help="default: <folder>/annotations.json")
    args = parser.parse_args()
    folder = args.folder
    grid, origin, res = S.read_map_yaml(folder/"map.yaml")
    semantic = P.build_semantic(grid, origin, res, P.load_annotations(args.annotations or folder/"annotations.json"))
    (folder/"semantic_map.json").write_text(json.dumps(semantic, indent=1, ensure_ascii=False), encoding="utf-8")
    topology_figure(grid, origin, res, semantic).save(folder/"topology.png")
    report = {"rooms": len(semantic["zones"]), "places": len(semantic["places"]), "doorways": len(semantic["edges"]),
              "anchors": len(semantic["anchors"]),
              "unnamed_areas": [p["label"] for p in semantic["places"] if p["kind"] == "area"]}
    truth_path = folder/"semantic_truth.json"
    if truth_path.is_file():
        truth = json.loads(truth_path.read_text(encoding="utf-8"))
        check = P.compare_with_truth(semantic, truth)
        (folder/"annotation_check.json").write_text(json.dumps(check, indent=1, ensure_ascii=False))
        report["check"] = {k: check[k] for k in ("room_iou_mean", "rooms_with_doorway", "doorway_error_m")}
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
