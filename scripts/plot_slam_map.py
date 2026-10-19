#!/usr/bin/env python3
"""Figure of a saved SLAM map package (launch/g1_slam_mapping.launch.py).

    bash scripts/g1_docker.sh python3 scripts/plot_slam_map.py maps/campus --out docs/images/slam

slam_map.png: (a) the 3-D point cloud coloured by height with the walked path, (b) the occupancy grid the VLN
uses, (c) with ground truth (simulation): the grid in the simulator frame against the true map.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory import slam_map as S  # noqa: E402

SS = 2
FONTS = {False: "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
         True: "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"}
PATH = (235, 110, 20)


def font(size, bold=False):
    try:
        return ImageFont.truetype(FONTS[bold], int(size*SS))
    except OSError:
        return ImageFont.load_default()


def read_pcd(path: Path) -> np.ndarray:
    data = path.read_bytes()
    end = data.index(b"DATA binary\n")+len(b"DATA binary\n")
    return np.frombuffer(data[end:], np.float32).reshape(-1, 3)


def turbo(t):
    """Google's Turbo colormap (polynomial approximation), t in 0..1 -> uint8 RGB."""
    t = np.clip(t, 0, 1)[:, None]
    c = np.array([[0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943],
                  [0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604],
                  [0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973]])
    powers = np.concatenate([t**k for k in range(6)], axis=1)
    return (np.clip(powers@c.T, 0, 1)*255).astype(np.uint8)


def cloud_view(cloud, traj, width=1500, elevation=40.0, azimuth=-60.0, top=4.0):
    """Oblique orthographic view of the cloud, painter's order, points coloured by height."""
    pts = cloud[(cloud[:, 2] < top)]
    a, e = math.radians(azimuth), math.radians(elevation)
    def project(p):
        u = p[:, 0]*math.cos(a)-p[:, 1]*math.sin(a)
        w = p[:, 0]*math.sin(a)+p[:, 1]*math.cos(a)
        return u, -w*math.sin(e)-p[:, 2]*math.cos(e), w*math.cos(e)-p[:, 2]*math.sin(e)
    u, v, depth = project(pts)
    scale = width*SS/(u.max()-u.min()+4)
    H = int((v.max()-v.min()+4)*scale)
    W = int(width*SS)
    cu = ((u-u.min()+2)*scale).astype(int)
    cv = ((v-v.min()+2)*scale).astype(int)
    order = np.argsort(-depth)  # far first, near points overwrite
    img = np.full((H, W, 3), 255, np.uint8)
    floor = pts[:, 2] < 0.12
    colors = turbo(np.clip(pts[:, 2]/3.0, 0, 1)*0.85+0.1)
    colors[floor] = (200, 204, 210)
    r = max(1, int(round(0.05*scale)))
    for du in range(-r, r+1):
        for dv in range(-r, r+1):
            uu, vv = np.clip(cu[order]+du, 0, W-1), np.clip(cv[order]+dv, 0, H-1)
            img[vv, uu] = colors[order]
    out = Image.fromarray(img)
    if traj is not None and len(traj):
        tu, tv, _ = project(np.c_[traj[:, :2], np.full(len(traj), 0.15)])
        line = list(zip(((tu-u.min()+2)*scale).tolist(), ((tv-v.min()+2)*scale).tolist()))
        d = ImageDraw.Draw(out)
        d.line(line, fill=PATH, width=3*SS, joint="curve")
        x, y = line[0]
        d.ellipse([x-8*SS, y-8*SS, x+8*SS, y+8*SS], fill=(30, 30, 30), outline=(255, 255, 255), width=2*SS)
    return out


def grid_view(grid, origin, res, traj, width):
    rgb = np.full(grid.shape+(3,), (214, 217, 222), np.uint8)
    rgb[grid == S.FREE] = (255, 255, 255)
    rgb[grid == S.OCCUPIED] = (35, 38, 45)
    img = Image.fromarray(np.flipud(rgb))
    scale = width*SS/img.width
    img = img.resize((int(img.width*scale), int(img.height*scale)), Image.NEAREST)
    if traj is not None and len(traj):
        px = lambda x, y: ((x-origin[0])/res*scale, (grid.shape[0]-(y-origin[1])/res)*scale)
        line = [px(x, y) for x, y in traj[:, :2]]
        d = ImageDraw.Draw(img)
        d.line(line, fill=PATH, width=3*SS, joint="curve")
        x, y = line[0]
        d.ellipse([x-8*SS, y-8*SS, x+8*SS, y+8*SS], fill=(30, 30, 30), outline=(255, 255, 255), width=2*SS)
    return img


def panel(img, tag, text):
    """The image under a title strip."""
    out = Image.new("RGB", (img.width, img.height+44*SS), (255, 255, 255))
    out.paste(img, (0, 44*SS))
    ImageDraw.Draw(out).text((14*SS, 22*SS), f"({tag}) {text}", fill=(30, 30, 30), font=font(22, True), anchor="lm")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT/"docs/images/slam")
    parser.add_argument("--font", default=FONTS[False])
    parser.add_argument("--font-bold", default=FONTS[True])
    args = parser.parse_args()
    FONTS[False], FONTS[True] = args.font, args.font_bold
    folder = args.folder
    meta = json.loads((folder/"meta.json").read_text())
    with (folder/"trajectory.csv").open() as f:
        traj = np.array([[float(r["x_m"]), float(r["y_m"]), float(r["z_m"])] for r in csv.DictReader(f)])
    grid, origin, res = S.read_map_yaml(folder/"map.yaml")
    panels = [panel(cloud_view(read_pcd(folder/"cloud.pcd"), traj), "a",
                    f"3-D point cloud, {meta['cloud_points']:,} voxels, coloured by height"),
              panel(grid_view(grid, origin, res, traj, 1500), "b", "occupancy grid (0.1 m) and the walked path")]
    if (folder/"map_world.yaml").is_file() and (folder/"comparison.png").is_file():
        comp = Image.open(folder/"comparison.png").convert("RGB")
        comp = comp.resize((1500*SS, int(comp.height*1500*SS/comp.width)), Image.NEAREST)
        m, t = meta.get("map", {}), meta.get("trajectory", {})
        panels.append(panel(comp, "c", f"vs ground truth: obstacle precision {m.get('obstacle_precision', 0):.0%}, "
                                       f"recall {m.get('obstacle_recall', 0):.0%}, path error {t.get('ate_rmse_m', 0):.2f} m"))
    W = max(p.width for p in panels)
    gap = 20*SS
    legend_h = 50*SS
    fig = Image.new("RGB", (W, sum(p.height for p in panels)+gap*(len(panels)-1)+legend_h), (255, 255, 255))
    y = 0
    for p in panels:
        fig.paste(p, ((W-p.width)//2, y))
        y += p.height+gap
    d = ImageDraw.Draw(fig)
    x = 20*SS
    yb = fig.height-legend_h//2
    entries = [("walked path", "line", PATH), ("start", "dot", (30, 30, 30)), ("free", "box", (255, 255, 255)),
               ("occupied", "box", (35, 38, 45)), ("unknown", "box", (214, 217, 222))]
    if len(panels) == 3:
        entries += [("SLAM only", "box", (220, 60, 50)), ("ground truth only", "box", (90, 140, 220))]
    for text, kind, col in entries:
        if kind == "line":
            d.line([(x, yb), (x+26*SS, yb)], fill=col, width=4*SS)
        elif kind == "dot":
            d.ellipse([x+5*SS, yb-9*SS, x+23*SS, yb+9*SS], fill=col)
        else:
            d.rectangle([x+3*SS, yb-10*SS, x+23*SS, yb+10*SS], fill=col, outline=(120, 120, 120))
        d.text((x+34*SS, yb), text, fill=(60, 60, 60), font=font(19), anchor="lm")
        x += int(d.textlength(text, font=font(19)))+70*SS
    args.out.mkdir(parents=True, exist_ok=True)
    fig = fig.resize((fig.width//SS, fig.height//SS), Image.LANCZOS)
    fig.save(args.out/"slam_map.png", optimize=True)
    print(json.dumps({"figure": str(args.out/"slam_map.png")}))


if __name__ == "__main__":
    main()
