#!/usr/bin/env python3
"""Figure of VLN episodes run in a map package's own frame (package_frame:=map, the localizer).

    bash scripts/g1_docker.sh python3 scripts/plot_localization.py outputs/maps/campus_dev \
        outputs/hvln_loc_tank outputs/hvln_loc_tank_drift --out docs/images/slam

localization.png, one row per episode: (left) the package's occupancy grid with the true path, the localised path and,
when /odom drifted, the odometry alone; (right) the position error of the localised pose over time.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/"scripts"))
from g1_factory import slam_map as S  # noqa: E402
from g1_factory.localization import compose, inverse  # noqa: E402
from g1_factory.map_package import world_from_map  # noqa: E402
from plot_slam_map import FONTS, SS, font, panel  # noqa: E402

TRUE, EST, ODOM = (60, 110, 200), (235, 110, 20), (150, 150, 150)


def read_rows(path: Path):
    with path.open() as f:
        return list(csv.DictReader(f))


def to_map(T, xy):
    return np.array([compose(T, (x, y, 0.0))[:2] for x, y in xy]) if len(xy) else np.zeros((0, 2))


def map_view(grid, origin, res, paths, width=900, margin_m=4.0):
    """Occupancy grid cropped to the paths, paths drawn on it (map frame)."""
    allxy = np.concatenate([p for p, _, _ in paths if len(p)])
    lo = np.maximum(np.floor((allxy.min(axis=0)-margin_m-np.array(origin))/res).astype(int), 0)
    hi = np.minimum(np.ceil((allxy.max(axis=0)+margin_m-np.array(origin))/res).astype(int), grid.shape[::-1])
    crop = grid[lo[1]:hi[1], lo[0]:hi[0]]
    rgb = np.full(crop.shape+(3,), (214, 217, 222), np.uint8)
    rgb[crop == S.FREE] = (255, 255, 255)
    rgb[crop == S.OCCUPIED] = (35, 38, 45)
    img = Image.fromarray(np.flipud(rgb))
    scale = width*SS/img.width
    img = img.resize((int(img.width*scale), int(img.height*scale)), Image.NEAREST)
    d = ImageDraw.Draw(img)
    px = lambda x, y: (((x-origin[0])/res-lo[0])*scale, (crop.shape[0]-((y-origin[1])/res-lo[1]))*scale)
    for xy, colour, w in paths:
        if len(xy) > 1:
            d.line([px(x, y) for x, y in xy], fill=colour, width=w*SS, joint="curve")
    x, y = px(*paths[0][0][0])
    d.ellipse([x-8*SS, y-8*SS, x+8*SS, y+8*SS], fill=(30, 30, 30), outline=(255, 255, 255), width=2*SS)
    return img


def error_chart(t, series, width=700, height=420, ymax=None, note=""):
    """Line chart of errors (m) over simulation time; series = [(values, colour, label)], note top right."""
    img = Image.new("RGB", (width*SS, height*SS), (255, 255, 255))
    d = ImageDraw.Draw(img)
    left, right, top, bottom = 70*SS, 20*SS, 30*SS, 50*SS
    W, H = img.width-left-right, img.height-top-bottom
    ymax = ymax or max(float(np.max(v)) for v, _, _ in series)*1.1
    t0, t1 = float(t[0]), float(t[-1])
    X = lambda v: left+(v-t0)/(t1-t0)*W
    Y = lambda v: top+H-min(v, ymax)/ymax*H
    step = [s for s in (0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20) if ymax/s <= 6][0]
    for k in range(int(ymax/step)+1):
        y = Y(k*step)
        d.line([(left, y), (left+W, y)], fill=(230, 230, 230), width=SS)
        d.text((left-8*SS, y), f"{k*step:g}", fill=(90, 90, 90), font=font(16), anchor="rm")
    tstep = [s for s in (10, 20, 50, 100, 200) if (t1-t0)/s <= 7][0]
    for k in range(int(t0//tstep)+1, int(t1//tstep)+1):
        d.text((X(k*tstep), top+H+8*SS), f"{k*tstep}", fill=(90, 90, 90), font=font(16), anchor="mt")
    d.text((left+W/2, img.height-6*SS), "simulation time (s)", fill=(60, 60, 60), font=font(17), anchor="mb")
    d.text((left-8*SS, top-6*SS), "m", fill=(60, 60, 60), font=font(17), anchor="rb")
    d.line([(left, top), (left, top+H), (left+W, top+H)], fill=(120, 120, 120), width=SS)
    for v, colour, label in series:
        d.line([(X(a), Y(b)) for a, b in zip(t, v)], fill=colour, width=3*SS, joint="curve")
    if note:
        d.text((left+W-6*SS, top+6*SS), note, fill=(110, 110, 110), font=font(17), anchor="rt")
    return img


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("package", type=Path)
    parser.add_argument("episodes", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, default=ROOT/"docs/images/slam")
    parser.add_argument("--font", default=FONTS[False])
    parser.add_argument("--font-bold", default=FONTS[True])
    args = parser.parse_args()
    FONTS[False], FONTS[True] = args.font, args.font_bold
    grid, origin, res = S.read_map_yaml(args.package/"map.yaml")
    W = world_from_map(args.package)
    map_from_world = inverse((W[0, 2], W[1, 2], math.atan2(W[1, 0], W[0, 0])))
    rows, tags = [], iter("abcdefgh")
    for episode in args.episodes:
        summary = json.loads((episode/"summary.json").read_text())
        launch = json.loads((episode/"launch_args.json").read_text())
        loc = read_rows(episode/"localization.csv")
        t = np.array([float(r["sim_time_s"]) for r in loc])
        err = np.array([float(r["position_error_m"]) for r in loc])
        est = to_map(map_from_world, [(float(r["x_m"]), float(r["y_m"])) for r in loc])
        truth = to_map(map_from_world, [(float(r["true_x_m"]), float(r["true_y_m"])) for r in loc])
        odom = [(float(r["odom_x_m"]), float(r["odom_y_m"])) for r in loc if r.get("odom_x_m")]
        paths = [(truth, TRUE, 5), (est, EST, 2)]
        series = [(err, EST, "localised")]
        drift = launch.get("odom_drift")
        if odom:
            odom_map = to_map(map_from_world, odom)
            paths.insert(0, (odom_map, ODOM, 2))
            odom_err = np.hypot(*(np.array(odom)-np.array([(float(r["true_x_m"]), float(r["true_y_m"])) for r in loc
                                                            if r.get("odom_x_m")])).T)
        s = summary["localization"]["position_error_m"]
        what = f"{summary['task']}, {summary['travelled_m']:.0f} m, " + (
            f"odometry drift {float(drift.split(',')[1]):g} deg/m and {float(drift.split(',')[0]):.0%}" if drift
            else "simulator odometry")
        left = panel(map_view(grid, origin, res, paths), next(tags), what)
        chart = error_chart(t, series, ymax=max(0.3, float(err.max())*1.2),
                            note=f"odometry alone: {odom_err[-1]:.1f} m off at the end" if odom else "")
        right = panel(chart, next(tags), f"localised vs truth: mean {s['mean']:.2f} m, max {s['max']:.2f} m")
        row = Image.new("RGB", (left.width+right.width+20*SS, max(left.height, right.height)), (255, 255, 255))
        row.paste(left, (0, 0))
        row.paste(right, (left.width+20*SS, 0))
        rows.append(row)
    W_ = max(r.width for r in rows)
    legend_h = 50*SS
    fig = Image.new("RGB", (W_, sum(r.height for r in rows)+20*SS*(len(rows)-1)+legend_h), (255, 255, 255))
    y = 0
    for r in rows:
        fig.paste(r, (0, y))
        y += r.height+20*SS
    d = ImageDraw.Draw(fig)
    x, yb = 20*SS, fig.height-legend_h//2
    for text, col, w in (("true path", TRUE, 5), ("localised in the map package", EST, 3), ("odometry alone", ODOM, 3)):
        d.line([(x, yb), (x+26*SS, yb)], fill=col, width=w*SS)
        d.text((x+34*SS, yb), text, fill=(60, 60, 60), font=font(19), anchor="lm")
        x += int(d.textlength(text, font=font(19)))+70*SS
    args.out.mkdir(parents=True, exist_ok=True)
    fig = fig.resize((fig.width//SS, fig.height//SS), Image.LANCZOS)
    fig.save(args.out/"localization.png", optimize=True)
    print(json.dumps({"figure": str(args.out/"localization.png")}))


if __name__ == "__main__":
    main()
