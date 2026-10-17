#!/usr/bin/env python3
"""Figures of one VLN episode: route map (decisions on the SLAM map) and a strip of decision frames.

    python3 scripts/plot_vln_episode.py results/isaac_pilot_2026-10-17/inspect_conveyor \
        --frames 2 4 inspection --out docs/images/vln --name conveyor

Reads summary.json + feedback.jsonl (+ decisions/*.jpg, inspection.jpg) written by the external_agent
evaluator. The route map draws the annotated regions, the start pose, every VLN decision at the pose
where it was taken (blue = move, grey = turn/look-around, orange = approach), the straight segments the
robot walked between them, the hidden target's footprint (evaluation only, red) and the final pose.
Run inside the g1-sim-ros2 image (needs the scene): bash scripts/g1_docker.sh python3 scripts/plot_vln_episode.py ...
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
from g1_factory.grounding import SemanticMap  # noqa: E402
from g1_factory.planning import OccupancyGrid  # noqa: E402
from g1_factory.scene import build_scene, load_config  # noqa: E402

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
COLORS = {"move": (31, 111, 180), "turn": (120, 120, 120), "approach": (230, 120, 20), "stop": (40, 150, 70)}
REGION_FILL = (238, 243, 247)


def font(size, bold=False):
    try:
        return ImageFont.truetype(BOLD if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


def load_episode(folder: Path):
    summary = json.loads((folder/"summary.json").read_text(encoding="utf-8"))
    events = [json.loads(line) for line in (folder/"feedback.jsonl").read_text(encoding="utf-8").splitlines() if line]
    decisions = [e for e in events if e.get("state") == "decision" and e.get("pose")]
    return summary, decisions


def route_map(summary, decisions, grid, regions, labels, footprints, scale=18, margin=4.5):
    start, final = summary["start_pose"][:2], summary["final_pose"][:2]
    pts = np.array([start, final]+[d["pose"][:2] for d in decisions]+[[b[0], b[1]] for b in footprints]
                   + [[b[2], b[3]] for b in footprints])
    x0, y0 = pts.min(axis=0)-margin
    x1, y1 = pts.max(axis=0)+margin
    to_px = lambda x, y: ((x-x0)*scale, (y1-y)*scale)

    # Background at grid resolution (regions, then mapped obstacles), scaled up with nearest neighbour.
    res = grid.resolution
    xs = np.arange(x0, x1, res)+res/2
    ys = np.arange(y1, y0, -res)-res/2
    gx, gy = np.meshgrid(xs, ys)
    rgb = np.full(gx.shape+(3,), 255, np.uint8)
    for bx0, by0, bx1, by1 in regions.values():
        rgb[(gx >= bx0) & (gx <= bx1) & (gy >= by0) & (gy <= by1)] = REGION_FILL
    row, col = grid.world_to_cell(np.stack([gx, gy], axis=-1))
    inside = grid.inside(row, col)
    blocked = np.zeros(gx.shape, bool)
    blocked[inside] = grid.data[row[inside], col[inside]] != 0
    rgb[blocked] = (150, 156, 162)
    img = Image.fromarray(rgb).resize((int((x1-x0)*scale), int((y1-y0)*scale)), Image.NEAREST)
    w, h = img.size
    draw = ImageDraw.Draw(img)

    for rid, (bx0, by0, bx1, by1) in regions.items():
        cx, cy = to_px((max(bx0, x0)+min(bx1, x1))/2, min(by1, y1))
        if bx1 > x0 and bx0 < x1 and by1 > y0 and by0 < y1:
            draw.text((cx, cy+8), labels.get(rid, rid), fill=(80, 100, 125), anchor="ma", font=font(20, True))
    for bx0, by0, bx1, by1 in footprints:
        a, b = to_px(bx0, by1), to_px(bx1, by0)
        draw.rectangle([a, b], outline=(210, 30, 30), width=5)

    path = [start]+[d["pose"][:2] for d in decisions]+[final]
    draw.line([to_px(*p) for p in path], fill=(31, 111, 180), width=5, joint="curve")
    sx, sy = to_px(*start)
    draw.ellipse([sx-12, sy-12, sx+12, sy+12], fill=(40, 40, 40))
    draw.text((sx+16, sy-12), "start", fill=(40, 40, 40), font=font(20, True))
    # Decisions taken on (nearly) the same spot - turning to look - share one marker labelled "4-8".
    clusters = []
    for k, d in enumerate(decisions, start=1):
        xy = np.asarray(d["pose"][:2], float)
        if clusters and np.linalg.norm(xy-clusters[-1]["xy"]) < 0.8:
            clusters[-1]["last"], clusters[-1]["actions"] = k, clusters[-1]["actions"]+[d.get("action")]
        else:
            clusters.append({"xy": xy, "first": k, "last": k, "actions": [d.get("action")]})
    for c in clusters:
        px, py = to_px(*c["xy"])
        action = ("approach" if "approach" in c["actions"] else
                  "turn" if "turn" in c["actions"] else c["actions"][-1])
        label = str(c["first"]) if c["first"] == c["last"] else f"{c['first']}-{c['last']}"
        r = 14 if len(label) < 3 else 22
        draw.rounded_rectangle([px-r, py-14, px+r, py+14], radius=14, fill=COLORS.get(action, (120, 120, 120)),
                               outline=(255, 255, 255), width=3)
        draw.text((px, py), label, fill=(255, 255, 255), anchor="mm", font=font(15, True))
    fx, fy = to_px(*final)
    draw.rectangle([fx-11, fy-11, fx+11, fy+11], fill=(40, 150, 70))
    draw.text((fx+16, fy-12), "stop", fill=(40, 150, 70), font=font(20, True))

    legend = [("move", COLORS["move"]), ("turn / look-around", COLORS["turn"]), ("approach", COLORS["approach"]),
              ("target (hidden from robot)", (210, 30, 30))]
    bar = Image.new("RGB", (max(w, 760), 40), (255, 255, 255))
    bdraw = ImageDraw.Draw(bar)
    lx = 12
    for text, color in legend:
        bdraw.rectangle([lx, 12, lx+18, 30], fill=color)
        bdraw.text((lx+26, 9), text, fill=(60, 60, 60), font=font(17))
        lx += 44+int(bdraw.textlength(text, font=font(17)))
    out = Image.new("RGB", (bar.width, h+40), (255, 255, 255))
    out.paste(img, (0, 0))
    out.paste(bar, (0, h))
    return out


def frame_strip(folder: Path, picks, decisions, width=400):
    files = sorted((folder/"decisions").glob("*.jpg"))
    tiles = []
    for pick in picks:
        if pick == "inspection":
            path, caption = folder/"inspection.jpg", "inspection frame"
        else:
            k = int(pick)
            path = files[k]
            d = decisions[k] if k < len(decisions) else {}
            caption = f"decision {k+1}: {d.get('action', '?')}"
            if d.get("plate") and len(d["plate"]) <= 16:   # a read plate; longer text is the second look
                caption += f" ({d['plate']})"
            elif "looked around" in (d.get("note") or ""):
                caption = f"decision {k+1}: look-around"
        tile = Image.open(path).convert("RGB")
        tile = tile.resize((width, round(tile.height*width/tile.width)))
        bar = Image.new("RGB", (width, 30), (255, 255, 255))
        ImageDraw.Draw(bar).text((8, 6), caption, fill=(40, 40, 40), font=font(16, True))
        canvas = Image.new("RGB", (width, tile.height+30), (255, 255, 255))
        canvas.paste(tile, (0, 0))
        canvas.paste(bar, (0, tile.height))
        tiles.append(canvas)
    gap = 8
    strip = Image.new("RGB", (sum(t.width for t in tiles)+gap*(len(tiles)-1), max(t.height for t in tiles)),
                      (255, 255, 255))
    x = 0
    for t in tiles:
        strip.paste(t, (x, 0))
        x += t.width+gap
    return strip


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("episode", type=Path, help="episode folder with summary.json and feedback.jsonl")
    parser.add_argument("--frames", nargs="*", default=[], help="decision indices (0-based) and/or 'inspection'")
    parser.add_argument("--out", type=Path, default=ROOT/"docs/images/vln")
    parser.add_argument("--name", default=None)
    args = parser.parse_args()
    summary, decisions = load_episode(args.episode)
    config = load_config(ROOT/"configs/factory.yaml")
    meta = build_scene(ROOT/"configs/factory.yaml", ROOT/"external/unitree_rl_gym", Path("/tmp/vln_plot/scene.xml"))
    grid = OccupancyGrid.from_scene(meta, 0.1)
    regions = {z["id"]: z["bounds"] for z in config["zones"]}
    labels = {z["id"]: z["label"] for z in config["zones"]}
    place = SemanticMap.load(config, ROOT/"configs/semantic_map.yaml").places[summary["expected_place"]]
    footprints = list(place.footprints) or [(place.object_xy[0]-.3, place.object_xy[1]-.3,
                                             place.object_xy[0]+.3, place.object_xy[1]+.3)]
    name = args.name or summary["task"]
    args.out.mkdir(parents=True, exist_ok=True)
    route_map(summary, decisions, grid, regions, labels, footprints).save(args.out/f"{name}_route.png", optimize=True)
    if args.frames:
        frame_strip(args.episode, args.frames, decisions).save(args.out/f"{name}_frames.jpg", quality=82)
    print(json.dumps({"route": str(args.out/f"{name}_route.png"), "decisions": len(decisions),
                      "frames": args.frames}))


if __name__ == "__main__":
    main()
