"""Zone and object recognition accuracy, independent of navigation.

A robot can receive an instruction anywhere, so before path planning it must know (a) which zone it is
in and (b) what it is looking at. This module samples evaluation poses over every place of the
topological map, computes ground truth from the semantic map (zone from the pose; visible objects by
projecting each object's 3-D box into the head camera and checking it against the depth image, so
occluded objects do not count), and scores VLM answers against it.

Capture runs in Isaac Sim (isaac_sim_node --capture); scoring runs anywhere (scripts/recognition_eval.py).
Camera: head RGB-D on the pelvis at (0.12, 0, 0.45) m, pitched 10 deg down, 640x480, HFOV 69.4 deg.
"""
from __future__ import annotations

import math
import re

import numpy as np

from g1_factory.semantic_world import buildings_from, place_of

CAMERA_XYZ, CAMERA_PITCH = (0.12, 0.0, 0.45), math.radians(10.0)
WIDTH, HEIGHT, HFOV = 640, 480, math.radians(69.4)
FX = WIDTH/(2*math.tan(HFOV/2))
INTRINSICS = (FX, FX, (WIDTH-1)/2, (HEIGHT-1)/2)
AISLE = "aisle"


def quat_matrix(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def map_from_optical(base_pos, base_quat_wxyz) -> np.ndarray:
    """4x4 pose of the camera optical frame (z forward, x right, y down) in the map, as published on /tf."""
    c, s = math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH)
    base = np.eye(4)
    base[:3, :3], base[:3, 3] = quat_matrix(base_quat_wxyz), base_pos
    link = np.eye(4)
    link[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])  # pitch down about y
    link[:3, 3] = CAMERA_XYZ
    optical = np.eye(4)
    optical[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    return base@link@optical


def zone_of(semantic: dict, xy) -> str:
    """Ground-truth answer to "which zone am I in": a zone id, or "aisle" outside all zones."""
    regions = {z["id"]: z["bounds"] for z in semantic["zones"]}
    for rid, (x0, y0, x1, y1) in regions.items():
        if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1:
            return rid
    return AISLE


def sample_poses(semantic: dict, meta: dict, per_place: int, seed: int = 0, clearance: float = 0.7) -> list:
    """per_place poses in every topological place (zones and aisle segments), random heading, free space."""
    from g1_factory.planning import CostMap, OccupancyGrid
    rng = np.random.default_rng(seed)
    cost = CostMap(OccupancyGrid.from_scene(meta, 0.1))
    grid = cost.grid
    rows, cols = np.nonzero(cost.distance >= clearance)
    xy = grid.cell_to_world(rows, cols)
    regions = {z["id"]: z["bounds"] for z in semantic["zones"]}
    buildings = buildings_from(meta)
    places = np.array([place_of(p, regions, buildings) for p in xy[::7]])  # thinned, then sampled
    pool = xy[::7]
    poses = []
    for place in sorted(set(places)):
        idx = np.nonzero(places == place)[0]
        for k in rng.choice(idx, size=min(per_place, len(idx)), replace=False):
            x, y = pool[k]+rng.uniform(-.05, .05, 2)
            poses.append({"place": str(place), "x": round(float(x), 3), "y": round(float(y), 3),
                          "yaw": round(float(rng.uniform(-math.pi, math.pi)), 4)})
    return poses


def _box_samples(aabb, n=6):
    lo, hi = np.asarray(aabb[:3], float), np.asarray(aabb[3:], float)
    margin = (hi-lo)*0.08
    axes = [np.linspace(lo[i]+margin[i], hi[i]-margin[i], n if hi[i]-lo[i] > .3 else 2) for i in range(3)]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return g


def visible_objects(objects: list, depth: np.ndarray, map_from_cam: np.ndarray, intrinsics=INTRINSICS,
                    max_range=10.0, min_box_px=24, min_points=4) -> list:
    """Objects actually seen: sample points of each 3-D box that project into the image AND match the depth
    there (not occluded). Returns [{id, class, label, bbox_px, distance, visible_points, status}] with status
    "visible" (counted for recall) or "marginal" (seen but small/occluded: neither required nor penalised)."""
    fx, fy, cx, cy = intrinsics
    cam_from_map = np.linalg.inv(map_from_cam)
    h, w = depth.shape
    out = []
    for o in objects:
        pts = _box_samples(o["aabb"])
        q = (cam_from_map@np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        z = q[:, 2]
        front = z > 0.3
        if not front.any():
            continue
        u = fx*q[front, 0]/z[front]+cx
        v = fy*q[front, 1]/z[front]+cy
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if not inside.any():
            continue
        zi = z[front][inside]
        ui, vi = u[inside].astype(int), v[inside].astype(int)
        measured = depth[vi, ui]
        seen = (measured > 0) & (np.abs(measured-zi) <= np.maximum(0.12, 0.05*zi))
        if seen.sum() == 0:
            continue
        us, vs = ui[seen], vi[seen]
        box = [int(us.min()), int(vs.min()), int(us.max()), int(vs.max())]
        dist = float(np.median(zi[seen]))
        big = (box[2]-box[0]) >= min_box_px and (box[3]-box[1]) >= min_box_px
        status = "visible" if (seen.sum() >= min_points and big and dist <= max_range) else "marginal"
        out.append({"id": o["id"], "class": o["class"], "label": o.get("label"), "bbox_px": box,
                    "distance": round(dist, 2), "visible_points": int(seen.sum()), "status": status})
    return out


def normalize_plate(text) -> str:
    return re.sub(r"[^A-Z0-9]+", " ", str(text or "").upper()).strip()


def box_point(depth: np.ndarray, box_px, map_from_cam: np.ndarray, intrinsics=INTRINSICS):
    """3-D map point of a detection: median depth of the box's central part (None without valid depth)."""
    fx, fy, cx, cy = intrinsics
    x1, y1, x2, y2 = box_px
    bw, bh = x2-x1, y2-y1
    patch = depth[int(max(0, y1+.3*bh)):int(min(depth.shape[0], y2-.3*bh+1)),
                  int(max(0, x1+.3*bw)):int(min(depth.shape[1], x2-.3*bw+1))]
    valid = patch[(patch > 0.1) & (patch < 30)]
    if valid.size < 5:
        return None
    z = float(np.median(valid))
    u, v = (x1+x2)/2, (y1+y2)/2
    return (map_from_cam@np.array([(u-cx)*z/fx, (v-cy)*z/fy, z, 1.0]))[:3]


def aabb_distance(point, aabb) -> float:
    lo, hi = np.asarray(aabb[:3]), np.asarray(aabb[3:])
    return float(np.linalg.norm(np.maximum(0, np.maximum(lo-point, point-hi))))


def iou(a, b) -> float:
    ix = max(0, min(a[2], b[2])-max(a[0], b[0]))
    iy = max(0, min(a[3], b[3])-max(a[1], b[1]))
    inter = ix*iy
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.0


def match_detections(predictions: list, truth: list, objects_by_id: dict, depth, map_from_cam,
                     radius=0.6, min_iou=0.3) -> dict:
    """Greedy matching of VLM detections to ground truth.

    A detection is a true positive when its class matches a seen object and either its 3-D point (depth at
    the box centre) lies within `radius` of that object's box, or (no valid depth) its box overlaps the
    object's projected box by `min_iou`. Matches to "marginal" objects are ignored (neither TP nor FP).
    """
    unmatched = {t["id"]: t for t in truth}
    tp, fp, wrong_class, ignored, errors, plates = [], [], [], [], [], []
    for p in predictions:
        point = box_point(depth, p["bbox_px"], map_from_cam)
        best, best_score = None, None
        for t in unmatched.values():
            if point is not None:
                d = aabb_distance(point, objects_by_id[t["id"]]["aabb"])
                ok, score = d <= radius, -d
            else:
                o = iou(p["bbox_px"], t["bbox_px"])
                ok, score = o >= min_iou, o
            if ok and (best is None or score > best_score):
                best, best_score = t, score
        if best is None:
            fp.append(p)
            continue
        if best["class"] != p["class"]:
            wrong_class.append({"predicted": p["class"], "truth": best["class"], "id": best["id"]})
            fp.append(p)
            continue
        unmatched.pop(best["id"])
        if best["status"] == "marginal":
            ignored.append(best["id"])
            continue
        tp.append(best["id"])
        if point is not None:
            errors.append(aabb_distance(point, objects_by_id[best["id"]]["aabb"]))
        if best.get("label"):
            plates.append({"id": best["id"], "truth": best["label"], "read": p.get("plate", ""),
                           "correct": normalize_plate(p.get("plate")) == normalize_plate(best["label"])})
    fn = [t["id"] for t in unmatched.values() if t["status"] == "visible"]
    return {"tp": tp, "fp": len(fp), "fn": fn, "wrong_class": wrong_class, "ignored": ignored,
            "loc_errors": errors, "plates": plates}
