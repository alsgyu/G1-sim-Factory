"""Map package: the robot's SLAM map plus what a person annotates on it, shared by the VLN and external agents.

maps/<name>/
  map.yaml, map.pgm, cloud.pcd, trajectory.csv, meta.json   the robot's SLAM map (g1_factory.ros2.slam_map_node)
  annotations.json   drawn by a person on the map (tools/map_annotator.html), map frame:
                     zones = name, aliases, kind (room / corridor), one or more rectangles, optional entrance;
                     anchors = named landmarks (crossings, docks, entrances)
  semantic_map.json  built here (build_semantic): rooms for the VLN, places and doorways (topology), anchors
Simulation only: map_world.yaml, evaluation.json (world_from_map), semantic_truth.json.

Topology is automatic: every free cell of the SLAM map gets the place of the annotated rectangle it lies in (rooms
before corridors), grown 0.6 m through free cells to cover rectangles drawn a little inside the walls; free space
that is still unlabelled and larger than 4 m2 becomes an "unnamed area" place (a room nobody named), and small gaps
join their neighbours. Two places are connected where their free cells touch; the doorway is the middle of
that contact. Walls and glass are occupied cells, so rooms behind a wall are never connected.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re

import numpy as np

from g1_factory import slam_map as S

FORMAT = "g1-map-annotations/1"


def slug(text: str, fallback: str) -> str:
    s = re.sub(r"[^0-9a-z]+", "_", text.lower()).strip("_")
    return s or fallback


def load_annotations(data) -> dict:
    """Validated annotations (dict or path): rectangles ordered, ids unique, bounds = union of rectangles."""
    if not isinstance(data, dict):
        data = json.loads(Path(data).read_text(encoding="utf-8"))
    zones, seen = [], set()
    for k, z in enumerate(data.get("zones", [])):
        rects = [[min(r[0], r[2]), min(r[1], r[3]), max(r[0], r[2]), max(r[1], r[3])] for r in z.get("rects", [])]
        rects = [r for r in rects if r[2]-r[0] > 0.2 and r[3]-r[1] > 0.2]
        if not rects:
            raise ValueError(f"zone {z.get('label') or k} has no rectangle")
        zid = z.get("id") or slug(z.get("label", ""), f"zone_{k+1}")
        if zid in seen or zid == "aisle":
            raise ValueError(f"zone id {zid!r} is used twice or reserved")
        seen.add(zid)
        kind = z.get("kind", "room")
        if kind not in ("room", "corridor"):
            raise ValueError(f"zone {zid}: kind must be room or corridor")
        r = np.array(rects)
        zones.append({"id": zid, "label": z.get("label") or zid, "aliases": [a for a in z.get("aliases", []) if a],
                      "kind": kind, "rects": [[round(float(v), 2) for v in q] for q in rects],
                      "bounds": [round(float(v), 2) for v in (*r[:, :2].min(axis=0), *r[:, 2:].max(axis=0))],
                      "entrance": [round(float(v), 2) for v in z["entrance"]] if z.get("entrance") else None})
    anchors = []
    for k, a in enumerate(data.get("anchors", [])):
        anchors.append({"id": a.get("id") or slug(a.get("label", ""), f"anchor_{k+1}"),
                        "label": a.get("label") or f"anchor {k+1}", "xy": [round(float(v), 2) for v in a["xy"][:2]]})
    return {"format": FORMAT, "frame": data.get("frame", "map"), "zones": zones, "anchors": anchors}


# ------------------------------------------------------------------ free-space labelling
def _cells(origin, res, shape, rect):
    c0 = max(0, int(math.floor((rect[0]-origin[0])/res)))
    r0 = max(0, int(math.floor((rect[1]-origin[1])/res)))
    c1 = min(shape[1], int(math.ceil((rect[2]-origin[0])/res)))
    r1 = min(shape[0], int(math.ceil((rect[3]-origin[1])/res)))
    return r0, r1, c0, c1


def _grow(labels, free, steps):
    """Spread labels into unlabelled free cells, one cell per step (4-neighbourhood)."""
    for _ in range(steps):
        before = (labels >= 0).sum()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            shifted = np.full_like(labels, -1)
            shifted[max(dr, 0):labels.shape[0]+min(dr, 0), max(dc, 0):labels.shape[1]+min(dc, 0)] = \
                labels[max(-dr, 0):labels.shape[0]+min(-dr, 0), max(-dc, 0):labels.shape[1]+min(-dc, 0)]
            take = (labels < 0) & free & (shifted >= 0)
            labels[take] = shifted[take]
        if (labels >= 0).sum() == before:
            break
    return labels


def _components(mask):
    """4-connected components of a boolean mask: (labels int32, count)."""
    comp = np.full(mask.shape, -1, np.int32)
    n = 0
    for r, c in zip(*np.nonzero(mask)):
        if comp[r, c] >= 0:
            continue
        stack = [(r, c)]
        comp[r, c] = n
        while stack:
            y, x = stack.pop()
            for yy, xx in ((y+1, x), (y-1, x), (y, x+1), (y, x-1)):
                if 0 <= yy < mask.shape[0] and 0 <= xx < mask.shape[1] and mask[yy, xx] and comp[yy, xx] < 0:
                    comp[yy, xx] = n
                    stack.append((yy, xx))
        n += 1
    return comp, n


def label_free_space(grid, origin, res, zones, slop_m=0.6, grow_m=3.0, min_area_m2=4.0):
    """(labels per cell, places): annotated places first, then unnamed areas for left-over free space."""
    free = grid == S.FREE
    labels = np.full(grid.shape, -1, np.int32)
    places = [{"id": z["id"], "label": z["label"], "kind": "zone" if z["kind"] == "room" else "aisle",
               "aliases": z.get("aliases", []), "rects": z["rects"]} for z in zones]
    order = sorted(range(len(zones)), key=lambda k: (zones[k]["kind"] == "room",
                                                     -sum((r[2]-r[0])*(r[3]-r[1]) for r in zones[k]["rects"])))
    for k in order:  # corridors first, then rooms from large to small: rooms and small rooms win overlaps
        for rect in zones[k]["rects"]:
            r0, r1, c0, c1 = _cells(origin, res, grid.shape, rect)
            block = labels[r0:r1, c0:c1]
            block[free[r0:r1, c0:c1]] = k
    labels = _grow(labels, free, int(round(slop_m/res)))  # rectangles drawn a little inside the walls
    comp, n = _components(free & (labels < 0))  # what is left: rooms nobody named (behind a door, ...)
    for c in range(n):
        cells = comp == c
        if cells.sum()*res*res < min_area_m2:
            continue
        labels[cells] = len(places)
        places.append({"id": f"area_{len(places)-len(zones)+1}", "label": f"Unnamed area {len(places)-len(zones)+1}",
                       "kind": "area", "aliases": [], "rects": []})
    labels = _grow(labels, free, int(round(grow_m/res)))  # small left-over gaps join their neighbours
    return labels, places


def contacts(labels, origin, res, min_cells=3):
    """{(a, b): (door xy, contact cells)} where the free cells of places a < b touch."""
    pairs = {}
    for axis in (0, 1):
        a = labels[:-1, :] if axis == 0 else labels[:, :-1]
        b = labels[1:, :] if axis == 0 else labels[:, 1:]
        mask = (a >= 0) & (b >= 0) & (a != b)
        rows, cols = np.nonzero(mask)
        x = origin[0]+(cols+0.5+(0.5 if axis == 1 else 0.0))*res
        y = origin[1]+(rows+0.5+(0.5 if axis == 0 else 0.0))*res
        for la, lb, px, py in zip(a[mask], b[mask], x, y):
            key = (int(min(la, lb)), int(max(la, lb)))
            pairs.setdefault(key, []).append((px, py))
    out = {}
    for key, pts in pairs.items():
        if len(pts) < min_cells:
            continue
        pts = np.array(pts)
        mean = pts.mean(axis=0)
        door = pts[np.argmin(np.linalg.norm(pts-mean, axis=1))]  # a contact point, so the doorway is on free floor
        out[key] = (door, len(pts))
    return out


def _center(labels, k, origin, res):
    rows, cols = np.nonzero(labels == k)
    if len(rows) == 0:
        return None
    pts = np.c_[origin[0]+(cols+0.5)*res, origin[1]+(rows+0.5)*res]
    mean = pts.mean(axis=0)
    return pts[np.argmin(np.linalg.norm(pts-mean, axis=1))]  # a cell of the place nearest its mean


def build_semantic(grid, origin, res, annotations: dict) -> dict:
    """semantic_map.json content in the map frame: rooms, places, doorways, anchors (schema of semantic_world)."""
    ann = load_annotations(annotations)
    labels, places = label_free_space(grid, origin, res, ann["zones"])
    for k, p in enumerate(places):
        c = _center(labels, k, origin, res)
        p["center"] = [round(float(v), 2) for v in c] if c is not None else None
        rows, cols = np.nonzero(labels == k)
        if len(rows):
            p["bounds"] = ([round(float(v), 2) for v in (origin[0]+cols.min()*res, origin[1]+rows.min()*res,
                                                          origin[0]+(cols.max()+1)*res, origin[1]+(rows.max()+1)*res)]
                           if p["kind"] != "zone" else next(z["bounds"] for z in ann["zones"] if z["id"] == p["id"]))
        p["free_m2"] = round(float(len(rows))*res*res, 1)
    edges = []
    for (a, b), (door, n) in sorted(contacts(labels, origin, res).items()):
        ca, cb = places[a]["center"], places[b]["center"]
        length = float(np.linalg.norm(np.array(ca)-door)+np.linalg.norm(door-np.array(cb))) if ca and cb else 0.0
        edges.append({"a": places[a]["id"], "b": places[b]["id"], "door": [round(float(v), 2) for v in door],
                      "width_m": round(n*res, 1), "length_m": round(length, 2)})
    index = {p["id"]: k for k, p in enumerate(places)}
    rooms = []
    for z in ann["zones"]:
        if z["kind"] != "room":
            continue
        entrance = z["entrance"] or _entrance(z, places, index, edges, labels, origin, res)
        rooms.append({"id": z["id"], "label": z["label"], "aliases": z["aliases"], "kind": "room",
                      "bounds": z["bounds"], "rects": z["rects"], "entrance": entrance})
    anchors = []
    for a in ann["anchors"]:
        r, c = int((a["xy"][1]-origin[1])/res), int((a["xy"][0]-origin[0])/res)
        inside = 0 <= r < labels.shape[0] and 0 <= c < labels.shape[1] and labels[r, c] >= 0
        anchors.append({**a, "place": places[labels[r, c]]["id"] if inside else None})
    h, w = grid.shape
    return {"frame": "map", "source": "annotations.json (person) + automatic topology from the SLAM map",
            "occupancy": {"bounds": [round(origin[0], 3), round(origin[1], 3), round(origin[0]+w*res, 3),
                                     round(origin[1]+h*res, 3)], "resolution": res},
            "zones": rooms, "places": places, "edges": edges, "anchors": anchors, "objects": [],
            "aisle_segments": {p["id"]: p["bounds"] for p in places if p["kind"] != "zone" and p.get("bounds")}}


def _entrance(zone, places, index, edges, labels, origin, res, inside_m=0.8):
    """Automatic entrance of a room: its widest doorway, moved a little into the room."""
    own = [e for e in edges if zone["id"] in (e["a"], e["b"])]
    center = places[index[zone["id"]]]["center"]
    if not own:
        return center
    door = np.array(max(own, key=lambda e: e["width_m"])["door"])
    step = np.array(center)-door
    point = door+step/max(np.linalg.norm(step), 1e-6)*inside_m if np.linalg.norm(step) > inside_m else door
    rows, cols = np.nonzero(labels == index[zone["id"]])
    pts = np.c_[origin[0]+(cols+0.5)*res, origin[1]+(rows+0.5)*res]
    return [round(float(v), 2) for v in pts[np.argmin(np.linalg.norm(pts-point, axis=1))]]


# ------------------------------------------------------------------ loading for the VLN
def world_from_map(folder: Path) -> np.ndarray:
    w = json.loads((Path(folder)/"evaluation.json").read_text())["world_from_map"]
    return S.se2(w["x"], w["y"], w["yaw_rad"])


def load_package(folder, frame="map"):
    """(grid, origin, resolution, semantic) of a map package.

    frame "map": as saved, the robot's own map frame (a real robot localises itself in this map).
    frame "world": simulation only, moved into the simulator frame with the start pose known from the simulator,
    which stands in for localisation.
    """
    folder = Path(folder)
    semantic = json.loads((folder/"semantic_map.json").read_text(encoding="utf-8"))
    if frame == "map":
        grid, origin, res = S.read_map_yaml(folder/"map.yaml")
        return grid, origin, res, semantic
    if frame != "world":
        raise ValueError("frame must be 'map' or 'world'")
    grid, origin, res = S.read_map_yaml(folder/"map_world.yaml")
    semantic = S.transform_semantic(semantic, world_from_map(folder))
    semantic["frame"] = "world (simulator frame; the start pose known from the simulator replaces localisation)"
    h, w = grid.shape
    semantic["occupancy"] = {"bounds": [origin[0], origin[1], origin[0]+w*res, origin[1]+h*res], "resolution": res}
    return grid, origin, res, semantic


def vln_inputs(semantic: dict):
    """regions, labels, entrances, anchors, aliases for the VLN nodes from a package semantic map."""
    rooms = [z for z in semantic["zones"] if z.get("kind", "room") == "room"]
    regions = {z["id"]: tuple(z["bounds"]) for z in rooms}
    labels = {z["id"]: z["label"] for z in rooms} | {"aisle": "aisle / corridor"}
    entrances = {z["id"]: tuple(z["entrance"][:2]) for z in rooms if z.get("entrance")}
    anchors = {a["label"]: tuple(a["xy"][:2]) for a in semantic.get("anchors", [])}
    aliases = {z["id"]: list(z.get("aliases", [])) for z in rooms}
    return regions, labels, entrances, anchors, aliases


# ------------------------------------------------------------------ simulation check against ground truth
def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2])-max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3])-max(a[1], b[1]))
    inter = ix*iy
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.0


def compare_with_truth(semantic_world: dict, truth: dict) -> dict:
    """Annotated rooms, automatic doorways and anchors (simulator frame) against the simulator's semantic map."""
    true_zones = {z["id"]: z for z in truth["zones"]}
    zones = {}
    for z in semantic_world["zones"]:
        t = true_zones.get(z["id"])
        zones[z["id"]] = round(iou(z["bounds"], t["bounds"]), 3) if t else None
    true_rooms = set(true_zones)
    room_edges = lambda edges, rooms: {(e["a"] if e["a"] in rooms else e["b"]): e["door"]
                                       for e in edges if (e["a"] in rooms) != (e["b"] in rooms)}
    ours = {}
    for e in semantic_world["edges"]:
        for room in (e["a"], e["b"]):
            if room in true_rooms and (e["b"] if room == e["a"] else e["a"]) not in true_rooms:
                ours.setdefault(room, []).append(e["door"])
    truth_doors = room_edges(truth["edges"], true_rooms)
    door_err = {}
    for room, door in truth_doors.items():
        if ours.get(room):
            door_err[room] = round(float(min(np.hypot(d[0]-door[0], d[1]-door[1]) for d in ours[room])), 2)
    room_pairs = lambda edges: {tuple(sorted((e["a"], e["b"]))) for e in edges
                                if e["a"] in true_rooms and e["b"] in true_rooms}
    true_anchors = {a["label"]: a["xy"] for a in truth.get("anchors", [])}
    anchor_err = {a["label"]: round(float(np.hypot(a["xy"][0]-true_anchors[a["label"]][0],
                                                   a["xy"][1]-true_anchors[a["label"]][1])), 2)
                  for a in semantic_world.get("anchors", []) if a["label"] in true_anchors}
    vals = [v for v in zones.values() if v is not None]
    return {"room_iou": zones, "room_iou_mean": round(float(np.mean(vals)), 3) if vals else None,
            "rooms_with_doorway": f"{len(door_err)}/{len(truth_doors)}",
            "doorway_error_m": door_err,
            "room_to_room_edges": {"annotated_map": sorted(room_pairs(semantic_world["edges"])),
                                   "truth": sorted(room_pairs(truth["edges"]))},
            "places": len(semantic_world["places"]), "edges": len(semantic_world["edges"]),
            "anchor_error_m": anchor_err}
