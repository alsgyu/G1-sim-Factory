#!/usr/bin/env python3
"""Plan instructions on a map package without ROS: grounding, topological route, A* path, and a figure of each.

    python3 scripts/plan_offline.py --start 0,0,0 \
        "Go to the operations control room and check whether the status indicator lights on the server rack are on."
    python3 scripts/plan_offline.py --start 24,-7.6,1.57 --tasks configs/instructions.yaml

Writes <out>/<n>_<target>.png (map, places on the route, doorways, path, start and goal) and <out>/plans.json.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from naive_pp.grounding import GroundingError, load_keywords  # noqa: E402
from naive_pp.package import FREE, OCCUPIED, load_package  # noqa: E402
from naive_pp.planner import NaivePlanner, PlannerConfig, PlanningError  # noqa: E402

COLORS = [(141, 211, 199), (255, 237, 140), (190, 186, 218), (251, 128, 114), (128, 177, 211), (253, 180, 98),
          (179, 222, 105), (252, 205, 229), (188, 128, 189), (204, 235, 197), (255, 210, 120), (160, 200, 230)]


def font(size, bold=False):
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def figure(pp: NaivePlanner, plan, title: str, scale=3):
    pkg = pp.pkg
    h, w = pkg.grid.shape
    rgb = np.full((h, w, 3), (226, 228, 232), np.uint8)
    rgb[pkg.grid == FREE] = (255, 255, 255)
    route = set(plan.route.places) if plan and plan.route else set()
    for k, p in enumerate(pkg.places):
        cells = (pkg.labels == k)
        tint = np.array(COLORS[k % len(COLORS)], float)
        alpha = 0.55 if p["id"] in route else 0.18
        rgb[cells] = (rgb[cells]*(1-alpha)+tint*alpha).astype(np.uint8)
    rgb[pkg.grid == OCCUPIED] = (55, 60, 70)
    img = Image.fromarray(np.flipud(rgb)).resize((w*scale, h*scale), Image.NEAREST)
    d = ImageDraw.Draw(img, "RGBA")
    px = lambda x, y: ((x-pkg.origin[0])/pkg.resolution*scale, (h-(y-pkg.origin[1])/pkg.resolution)*scale)
    for p in pkg.places:
        if p.get("center") and p["kind"] != "area":
            d.text(px(*p["center"]), p["label"], fill=(30, 30, 30), anchor="mm", font=font(13, True),
                   stroke_width=3, stroke_fill=(255, 255, 255))
    for e in pkg.semantic.get("edges", []):
        x, y = px(*e["door"])
        d.ellipse([x-4, y-4, x+4, y+4], fill=(255, 255, 255), outline=(60, 60, 60), width=2)
    if plan is not None:
        d.line([px(*q) for q in plan.points], fill=(220, 60, 30, 255), width=4)
        for door in plan.route.doors if plan.route else []:
            x, y = px(*door)
            d.ellipse([x-6, y-6, x+6, y+6], fill=(255, 200, 0), outline=(40, 40, 40), width=2)
        sx, sy = px(*plan.start[:2])
        d.ellipse([sx-8, sy-8, sx+8, sy+8], fill=(40, 120, 220), outline=(255, 255, 255), width=2)
        gx, gy, gyaw = plan.goal
        x, y = px(gx, gy)
        d.ellipse([x-9, y-9, x+9, y+9], fill=(30, 160, 70), outline=(255, 255, 255), width=2)
        d.line([(x, y), (x+22*math.cos(gyaw), y-22*math.sin(gyaw))], fill=(30, 160, 70), width=4)
    d.rectangle([0, 0, img.width, 34], fill=(255, 255, 255, 220))
    d.text((8, 17), title, fill=(20, 20, 20), anchor="lm", font=font(15, True))
    return img


def parse_pose(text):
    v = [float(s) for s in text.replace(",", " ").split()]
    if len(v) != 3:
        raise argparse.ArgumentTypeError("pose is x,y,yaw (map frame, m and rad)")
    return tuple(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("instructions", nargs="*")
    ap.add_argument("--package", type=Path, default=HERE/"maps/campus_dev")
    ap.add_argument("--keywords", type=Path, default=HERE/"configs/place_keywords.yaml")
    ap.add_argument("--start", type=parse_pose, default=(0.0, 0.0, 0.0), help="robot pose x,y,yaw in the map frame")
    ap.add_argument("--tasks", type=Path, help="YAML with tasks: {name: {instruction, start: [x, y, yaw]}}")
    ap.add_argument("--goal-mode", default="center", choices=("center", "entrance"))
    ap.add_argument("--out", type=Path, default=HERE/"outputs/offline")
    args = ap.parse_args()
    jobs = [(f"{k+1:02d}", text, args.start) for k, text in enumerate(args.instructions)]
    if args.tasks:
        tasks = yaml.safe_load(args.tasks.read_text(encoding="utf-8"))["tasks"]
        jobs += [(name, t["instruction"] if isinstance(t["instruction"], str) else json.dumps(t["instruction"],
                                                                                              ensure_ascii=False),
                  tuple(t.get("start", args.start))) for name, t in tasks.items()]
    if not jobs:
        ap.error("give an instruction or --tasks")
    pp = NaivePlanner(load_package(args.package), PlannerConfig(goal_mode=args.goal_mode), load_keywords(args.keywords))
    args.out.mkdir(parents=True, exist_ok=True)
    results = []
    for name, text, start in jobs:
        row = {"name": name, "instruction": text, "start": list(start)}
        plan = None
        try:
            g = pp.ground(text)
            row["grounding"] = {"target": g.target.label if g.target else None, "match": g.target.match if g.target else None,
                                "matched": g.target.matched if g.target else None,
                                "waypoints": [w.label for w in g.waypoints], "alternatives": g.alternatives,
                                "cancel": g.cancel, "error": g.error}
            if g.ok:
                plan = pp.plan(start, g.target, g.waypoints)
                row["plan"] = plan.summary()
        except (GroundingError, PlanningError) as exc:
            row["error"] = str(exc)
        results.append(row)
        label = row.get("grounding", {}).get("target") or "none"
        title = f"{name}: {text[:90]}" if plan is None else \
            f"{name}: {label}  |  {plan.length_m:.1f} m, {plan.search}  |  {plan.route.text() if plan.route else ''}"
        figure(pp, plan, title).save(args.out/f"{name}_{label.replace(' ', '_').lower()}.png")
        print(json.dumps(row, ensure_ascii=False))
    (args.out/"plans.json").write_text(json.dumps(results, indent=1, ensure_ascii=False)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
