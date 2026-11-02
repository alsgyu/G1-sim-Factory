#!/usr/bin/env python3
"""Carry a person's annotations from one SLAM map of the campus to another one, then build its map package.

    python3 naive-pp/scripts/transfer_annotations.py --from naive-pp/maps/campus_dev maps/campus

A new mapping run (another simulator, another day, another start pose) gives a new map frame. Instead of drawing
the rooms again, this registers the annotated map to the new one and moves the annotations with it:
  1. the obstacle cells of the annotated map become a point set;
  2. for each of the four wall-aligned orientations (SLAM maps here are turned so walls run along x and y), the
     point set is centred on the new map's obstacle cells and matched to them with the localizer's correlative scan
     matcher (likelihood field, coarse window then fine refinement);
  3. the best orientation wins if most points land on the new map's obstacles (inliers), else nothing is written;
  4. rectangles, entrances and anchors are moved by that transform into <target>/annotations.json, and the
     place graph is rebuilt from the new map (semantic_map.json, topology.png).
Check topology.png by eye before using the package, as after drawing by hand.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO/"scripts"))
from g1_factory import map_package as P  # noqa: E402
from g1_factory import slam_map as S  # noqa: E402
from g1_factory.localization import ScanMatcher  # noqa: E402
from build_map_package import topology_figure  # noqa: E402


def obstacle_points(grid, origin, res, n=3000, seed=0):
    rows, cols = np.nonzero(grid == S.OCCUPIED)
    pts = np.c_[origin[0]+(cols+0.5)*res, origin[1]+(rows+0.5)*res]
    if len(pts) > n:
        pts = pts[np.random.default_rng(seed).choice(len(pts), n, replace=False)]
    return pts


def register(src, dst, window_m=3.0, window_deg=4.0):
    """(x, y, yaw) of the transform dst <- src, inlier share and score; src/dst = (grid, origin, res)."""
    pts = obstacle_points(*src)
    target = obstacle_points(*dst, n=20000)
    matcher = ScanMatcher(*dst)
    best = None
    for k in range(4):
        yaw = k*math.pi/2
        c, s = math.cos(yaw), math.sin(yaw)
        turned = pts@np.array([[c, s], [-s, c]])
        lo, hi = np.percentile(turned, [2, 98], axis=0)
        tlo, thi = np.percentile(target, [2, 98], axis=0)
        shift = (tlo+thi)/2-(lo+hi)/2  # middles of the two maps' extents (robust to stray cells)
        pivot = np.zeros(2)
        m = matcher.match(pts, pivot, (shift[0], shift[1], yaw), window_m, math.radians(window_deg), peaks=3)
        cand = (m.x, m.y, m.yaw, m.inliers, m.score)
        if best is None or cand[4] > best[4]:
            best = cand
    return best


def transform_annotations(ann: dict, T, target_map: dict) -> dict:
    x, y, yaw = T
    c, s = math.cos(yaw), math.sin(yaw)
    pt = lambda p: [round(x+c*p[0]-s*p[1], 2), round(y+s*p[0]+c*p[1], 2)]

    def rect(r):
        corners = np.array([pt((r[0], r[1])), pt((r[2], r[1])), pt((r[2], r[3])), pt((r[0], r[3]))])
        return [*corners.min(axis=0).round(2).tolist(), *corners.max(axis=0).round(2).tolist()]
    out = json.loads(json.dumps(ann))
    out["map"] = target_map
    for z in out.get("zones", []):
        z["rects"] = [rect(r) for r in z.get("rects", [])]
        if z.get("entrance"):
            z["entrance"] = pt(z["entrance"])
    for a in out.get("anchors", []):
        a["xy"] = pt(a["xy"])
    out["note"] = (f"Moved from another annotated map by scripts/transfer_annotations.py (x {x:.2f} m, y {y:.2f} m, "
                   f"yaw {math.degrees(yaw):.2f} deg); check topology.png. Original note: {ann.get('note', '')}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", type=Path, help="map package folder with the new map.yaml")
    ap.add_argument("--from", dest="source", type=Path, default=HERE/"maps/campus_dev",
                    help="annotated map package (map.yaml + annotations.json)")
    ap.add_argument("--min-inliers", type=float, default=0.6)
    args = ap.parse_args()
    src = S.read_map_yaml(args.source/"map.yaml")
    dst = S.read_map_yaml(args.target/"map.yaml")
    x, y, yaw, inliers, score = register(src, dst)
    result = {"target_from_source": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(math.degrees(yaw), 2)},
              "inliers": round(inliers, 3), "score": round(score, 3)}
    if inliers < args.min_inliers:
        print(json.dumps({**result, "error": "the maps do not match well enough; annotate the new map by hand "
                                             "(tools/map_annotator.html)"}))
        sys.exit(1)
    ann = json.loads((args.source/"annotations.json").read_text(encoding="utf-8"))
    grid, origin, res = dst
    moved = transform_annotations(ann, (x, y, yaw), {"image": "map.pgm", "resolution": res,
                                                     "origin": [round(origin[0], 3), round(origin[1], 3)]})
    (args.target/"annotations.json").write_text(json.dumps(moved, indent=1, ensure_ascii=False)+"\n", encoding="utf-8")
    semantic = P.build_semantic(grid, origin, res, P.load_annotations(moved))
    (args.target/"semantic_map.json").write_text(json.dumps(semantic, indent=1, ensure_ascii=False), encoding="utf-8")
    topology_figure(grid, origin, res, semantic).save(args.target/"topology.png")
    result.update(rooms=len(semantic["zones"]), places=len(semantic["places"]), doorways=len(semantic["edges"]),
                  unnamed_areas=[p["label"] for p in semantic["places"] if p["kind"] == "area"])
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
