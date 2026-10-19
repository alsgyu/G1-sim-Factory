"""Semantic and topological map of the campus: the layers a robot keeps on top of its SLAM map.

* Geometric layer: the occupancy grid (/map) - what SLAM produces.
* Semantic layer: objects (class, name-plate label, zone, 3-D box) and zones (named regions).
* Topological layer: places (zones and aisle segments) as graph nodes, traversable connections as
  edges with the doorway/opening point between them, and annotated anchors attached to places.

Here the layers are built from the simulator's ground truth, i.e. what a person would annotate on a
SLAM map. The VLN only receives the zone/anchor part (see configs/semantic_map.yaml); the object layer is
the evaluation ground truth for recognition accuracy (scripts/recognition_eval.py).
"""
from __future__ import annotations

from collections import defaultdict
import heapq
import math
import re

import numpy as np

OBJECT_RULES = (
    # (regex on the geom name, class, plate label from the first group or None)
    (r"factory_shelf_material_storage_(r\d+)_", "storage rack", None),
    (r"factory_staging_(?:pallet|strap)_(\d+)", "pallet", None),
    (r"factory_assembly_conveyor_(\d+)_(?!plate)", "conveyor", lambda i: f"CONVEYOR {'AB'[int(i)]}"),
    (r"factory_assembly_bench_(\d+)_(?!plate)", "workbench", lambda i: f"WORKBENCH {int(i)+1}"),
    (r"factory_assembly_machine_(?:control_|window_)?(\d+)$", "CNC machine", lambda i: f"CNC MACHINE {int(i)+1}"),
    (r"factory_inspection_station_(\d+)_", "inspection station", lambda i: f"STATION {int(i)+1}"),
    (r"factory_inspection_bin_(\d+)$", "parts bin", None),
    (r"factory_inspection_cabinet_(\d+)$", "metrology cabinet", lambda i: "METROLOGY"),
    (r"factory_charger_(?:light_|screen_)?(\d+)$", "charging dock", lambda i: f"CHARGER {int(i)+1}"),
    (r"factory_amr_(\d+)_", "AMR cart", None),
    (r"factory_battery_cabinet_(\d+)$", "battery cabinet", lambda i: "BATTERY STORAGE"),
    (r"factory_(fire_cabinet_west)(?:_window|_extinguisher.*)?$", "fire extinguisher cabinet",
     lambda i: "FIRE EXTINGUISHER"),
    (r"factory_(electrical_panel)$", "electrical panel", lambda i: "ELECTRICAL PANEL"),
    (r"factory_office_workstation_(\d+_\d+)_(?:top|leg_.*|monitor_.*|keyboard|mouse|notebook)$", "desk", None),
    (r"factory_office_(?:workstation|meeting|control)_(?:chair_)?(\w+?)_(?:chair_)?(?:arm_.*|back|pedestal|seat)$",
     "office chair", None),
    (r"factory_office_workstation_(bookshelf)(?:_books)?", "bookshelf", None),
    (r"factory_office_workstation_(books)_\d+$", "bookshelf", None),
    (r"factory_office_(\w+?)_plant_(?:leaf_\d+|pot|stem)$", "plant", None),
    (r"factory_office_meeting_(table)_(?:top|support_.*)$", "meeting table", None),
    (r"factory_office_meeting_(display)(?:_frame|_line_\d+)?$", "wall display", None),
    (r"factory_office_lounge_(sofa)_", "sofa", None),
    (r"factory_office_lounge_(armchair)_", "sofa", None),
    (r"factory_office_lounge_(coffee_table|table_leg)", "coffee table", None),
    (r"factory_office_lounge_(vending)(?:_window|_item_\d+_\d+)?$", "vending machine", None),
    (r"factory_office_control_server_(?:led_|slot_)?(\d+)(?:_\d+)?$", "server rack", lambda i: f"SERVER RACK {int(i)+1}"),
    (r"factory_office_control_console_(\d+)_", "control console", None),
    (r"factory_office_(fire_cabinet)(?:_window|_extinguisher.*)?$", "fire extinguisher cabinet",
     lambda i: "FIRE EXTINGUISHER"),
    (r"factory_utility_(pump)_(?:volute.*|foot|bearing_housing)$", "centrifugal pump", lambda i: "PUMP P-101"),
    (r"factory_utility_(motor)_", "electric motor", lambda i: "MOTOR M-101"),
    (r"factory_utility_(coupling_guard)$", "coupling guard", None),
    (r"factory_utility_(suction_valve|discharge_valve|tank_valve)_", "valve", None),
    (r"factory_utility_(discharge_gauge|tank_gauge)_", "pressure gauge", None),
    (r"factory_utility_(tank)_(?:shell|head_.*|leg_\d+)$", "pressure tank", lambda i: "TK-101"),
    (r"factory_utility_(mcc)_(?:cabinet|door_seam_\d+|lamp_\d+)$", "electrical cabinet", lambda i: "MCC-1"),
    (r"factory_utility_(rack)_", "parts rack", None),
    (r"factory_utility_bollard_(\d+)$", "bollard", None),
)
_RULES = [(re.compile(p), cls, label) for p, cls, label in OBJECT_RULES]


def region_of(regions: dict, xy) -> str:
    x, y = float(xy[0]), float(xy[1])
    for rid, (x0, y0, x1, y1) in regions.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return rid
    return "aisle"


def _geom_aabb(model, data, gid):
    """World axis-aligned box of one geom, or None for planes/meshes."""
    import mujoco
    t = model.geom_type[gid]
    size = model.geom_size[gid]
    if t == mujoco.mjtGeom.mjGEOM_BOX or t == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
        half = size[:3]
    elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        half = np.array([size[0], size[0], size[1]])
    elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
        half = np.array([size[0], size[0], size[1]+size[0]])
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        half = np.full(3, size[0])
    else:
        return None
    rot = data.geom_xmat[gid].reshape(3, 3)
    extent = np.abs(rot)@half
    center = data.geom_xpos[gid]
    return np.concatenate([center-extent, center+extent])


def scene_objects(model, data, regions: dict) -> list:
    """Group the scene's geoms into labelled objects (class, plate label, zone, 3-D AABB)."""
    import mujoco
    groups = defaultdict(lambda: {"boxes": []})
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        for pattern, cls, label in _RULES:
            match = pattern.match(name)
            if match:
                key = (cls, match.group(1))
                box = _geom_aabb(model, data, gid)
                if box is not None:
                    groups[key]["boxes"].append(box)
                    groups[key]["label"] = label(match.group(1)) if label else None
                break
    objects = []
    for (cls, key), g in groups.items():
        boxes = np.array(g["boxes"])
        aabb = np.concatenate([boxes[:, :3].min(axis=0), boxes[:, 3:].max(axis=0)])
        center = (aabb[:3]+aabb[3:])/2
        objects.append({"id": f"{cls.replace(' ', '_')}_{key}".lower(), "class": cls, "label": g["label"],
                        "region": region_of(regions, center[:2]),
                        "center": [round(float(v), 3) for v in center],
                        "aabb": [round(float(v), 3) for v in aabb]})
    return sorted(objects, key=lambda o: (o["class"], o["id"]))


def place_of(xy, regions: dict, buildings: dict) -> str:
    """Topological place: a zone, or the aisle segment of the building the point is in."""
    rid = region_of(regions, xy)
    if rid != "aisle":
        return rid
    for name, (x0, y0, x1, y1) in buildings.items():
        if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1:
            return name
    return "bridge"


def topological_map(roadmap, regions: dict, labels: dict, buildings: dict, anchors: dict) -> dict:
    """Places = zones + aisle segments; edges = lattice connections between them, with the opening point.

    buildings: aisle segments by rectangle, e.g. {"warehouse_aisle": warehouse bounds, ...}; aisle lattice
    nodes outside all of them belong to "bridge". anchors: {label: (x, y)} annotated landmarks.
    """
    nodes = roadmap.nodes
    place = [place_of(p, regions, buildings) for p in nodes]
    members = defaultdict(list)
    for i, pid in enumerate(place):
        members[pid].append(i)
    crossings = defaultdict(list)
    for a, neighbours in enumerate(roadmap.edges()):  # adjacency lists
        for b, _ in neighbours:
            if a < b and place[a] != place[b]:
                crossings[tuple(sorted((place[a], place[b])))].append((nodes[a]+nodes[b])/2)
    aisle_labels = {"warehouse_aisle": "Warehouse main aisle", "office_corridor": "Office corridor",
                    "pump_passage": "Pump room passage", "bridge": "Warehouse-office bridge"}
    places = []
    for pid, idx in sorted(members.items()):
        pts = nodes[idx]
        if pid in regions:
            center = [(regions[pid][0]+regions[pid][2])/2, (regions[pid][1]+regions[pid][3])/2]
        else:
            center = pts.mean(axis=0).tolist()
        places.append({"id": pid, "label": labels.get(pid, aisle_labels.get(pid, pid)),
                       "kind": "zone" if pid in regions else "aisle", "center": [round(float(v), 2) for v in center],
                       "bounds": regions.get(pid), "lattice_nodes": len(idx)})
    centers = {p["id"]: np.array(p["center"]) for p in places}
    edges = []
    for (a, b), mids in sorted(crossings.items()):
        door = np.mean(mids, axis=0)
        length = float(np.linalg.norm(centers[a]-door)+np.linalg.norm(door-centers[b]))
        edges.append({"a": a, "b": b, "door": [round(float(v), 2) for v in door], "length_m": round(length, 2),
                      "crossings": len(mids)})
    anchor_list = [{"label": name, "xy": [round(float(v), 2) for v in xy],
                    "place": place_of(xy, regions, buildings)} for name, xy in anchors.items()]
    return {"places": places, "edges": edges, "anchors": anchor_list}


def route(topo: dict, start: str, goal: str) -> list:
    """Shortest place sequence on the topological graph (Dijkstra over edge lengths)."""
    graph = defaultdict(list)
    for e in topo["edges"]:
        graph[e["a"]].append((e["b"], e["length_m"]))
        graph[e["b"]].append((e["a"], e["length_m"]))
    best, queue, previous = {start: 0.0}, [(0.0, start)], {}
    while queue:
        d, node = heapq.heappop(queue)
        if node == goal:
            break
        if d > best.get(node, math.inf):
            continue
        for nxt, w in graph[node]:
            if d+w < best.get(nxt, math.inf):
                best[nxt], previous[nxt] = d+w, node
                heapq.heappush(queue, (d+w, nxt))
    if goal not in best:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    return path[::-1]


def buildings_from(meta: dict) -> dict:
    """Aisle segments of the campus by building footprint."""
    out = {"warehouse_aisle": meta["warehouse_bounds"], "office_corridor": meta["office_bounds"]}
    if meta.get("utility_bounds"):
        out["pump_passage"] = [38.0, -16.2, 42.0, -11.8]
    return out


def build_semantic_map(model, data, meta: dict, anchors: dict, resolution: float = 0.1) -> dict:
    """All three layers in one JSON-serialisable dict (frame: map, metres)."""
    from g1_factory.planning import CostMap, OccupancyGrid
    from g1_factory.roadmap import RoadMap
    regions = {zid: z["bounds"] for zid, z in meta["zones"].items()}
    labels = {zid: z["label"] for zid, z in meta["zones"].items()}
    roadmap = RoadMap(CostMap(OccupancyGrid.from_scene(meta, resolution)), regions)
    topo = topological_map(roadmap, regions, labels, buildings_from(meta), anchors)
    zones = [{"id": zid, "label": z["label"], "kind": z.get("kind"), "bounds": z["bounds"], "entrance": z.get("goal")}
             for zid, z in meta["zones"].items()]
    return {"frame": "map", "units": "m", "source": "simulator ground truth (= human annotation of the SLAM map)",
            "occupancy": {"bounds": meta["bounds"], "resolution": resolution},
            "aisle_segments": buildings_from(meta),
            "zones": zones, "objects": scene_objects(model, data, regions), **topo}


CATEGORY_COLORS = {  # object classes grouped for the figure
    "storage": ((150, 110, 60), {"storage rack", "pallet", "parts rack", "parts bin", "bookshelf"}),
    "production": ((40, 110, 170), {"conveyor", "workbench", "CNC machine", "inspection station", "metrology cabinet"}),
    "energy": ((210, 140, 20), {"charging dock", "AMR cart", "battery cabinet", "electrical panel", "electrical cabinet"}),
    "pump": ((20, 80, 200), {"centrifugal pump", "electric motor", "coupling guard", "valve", "pressure gauge",
                             "pressure tank", "bollard"}),
    "safety": ((210, 30, 30), {"fire extinguisher cabinet"}),
    "office": ((110, 110, 130), {"desk", "office chair", "plant", "meeting table", "wall display", "sofa",
                                 "coffee table", "vending machine", "server rack", "control console"}),
}


def draw_semantic_map(semantic: dict, grid, scale: float = 14.0):
    """Top-down figure of the three layers: occupancy, zones/objects, place graph."""
    from PIL import Image, ImageDraw, ImageFont
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 13)
        small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
    except OSError:
        font = small = ImageFont.load_default()
    gx0, gy0, gx1, gy1 = semantic["occupancy"]["bounds"]
    pad = 3.0  # metres around the map, so edge labels stay readable
    x0, y0, x1, y1 = gx0-pad, gy0-pad, gx1+pad, gy1+pad
    w, h = int((x1-x0)*scale), int((y1-y0)*scale)
    px = lambda x, y: ((x-x0)*scale, (y1-y)*scale)
    occ = np.flipud(grid.data)
    rgb = np.full(occ.shape+(3,), 255, np.uint8)
    rgb[occ == 0] = (246, 247, 248)
    rgb[occ > 0] = (150, 155, 160)
    img = Image.new("RGB", (w, h), (255, 255, 255))
    img.paste(Image.fromarray(rgb).resize((int((gx1-gx0)*scale), int((gy1-gy0)*scale)), Image.NEAREST),
              (int(pad*scale), int(pad*scale)))
    draw = ImageDraw.Draw(img, "RGBA")
    for z in semantic["zones"]:
        bx0, by0, bx1, by1 = z["bounds"]
        draw.rectangle([px(bx0, by1), px(bx1, by0)], fill=(70, 130, 180, 28), outline=(70, 110, 150, 160), width=2)
    color_of = {cls: col for col, classes in CATEGORY_COLORS.values() for cls in classes}
    for o in semantic["objects"]:
        ax0, ay0, _, ax1, ay1, _ = o["aabb"]
        col = color_of.get(o["class"], (90, 90, 90))
        draw.rectangle([px(ax0, ay1), px(ax1, ay0)], fill=(*col, 170), outline=(*col, 255))
        if o["label"]:
            cx, cy = px(*o["center"][:2])
            draw.text((cx, cy), o["label"], fill=(20, 20, 20), anchor="mm", font=small)
    centers = {p["id"]: p["center"] for p in semantic["places"]}
    for e in semantic["edges"]:
        pts = [px(*centers[e["a"]]), px(*e["door"]), px(*centers[e["b"]])]
        draw.line(pts, fill=(230, 90, 20, 230), width=3)
        dx, dy = px(*e["door"])
        draw.rectangle([dx-5, dy-5, dx+5, dy+5], fill=(255, 255, 255), outline=(230, 90, 20), width=2)
    for p in semantic["places"]:
        cx, cy = px(*p["center"])
        r = 9 if p["kind"] == "zone" else 7
        draw.ellipse([cx-r, cy-r, cx+r, cy+r], fill=(230, 90, 20) if p["kind"] == "zone" else (255, 160, 60),
                     outline=(255, 255, 255), width=2)
        draw.text((cx, cy+r+3), p["label"], fill=(30, 40, 60), anchor="ma", font=font)
    for a in semantic["anchors"]:
        ax, ay = px(*a["xy"])
        draw.polygon([(ax, ay-8), (ax+7, ay+5), (ax-7, ay+5)], fill=(40, 150, 70))
    legend = [("zone", (70, 130, 180)), ("place / edge / door", (230, 90, 20)), ("annotated anchor", (40, 150, 70))] + \
        [(name, col) for name, (col, _) in CATEGORY_COLORS.items()]
    bar = Image.new("RGB", (w, 30), (255, 255, 255))
    bd = ImageDraw.Draw(bar)
    lx = 8
    for text, col in legend:
        bd.rectangle([lx, 9, lx+14, 23], fill=col)
        bd.text((lx+19, 8), text, fill=(50, 50, 50), font=font)
        lx += 34+int(bd.textlength(text, font=font))
    out = Image.new("RGB", (w, h+30), (255, 255, 255))
    out.paste(img, (0, 0))
    out.paste(bar, (0, h))
    return out


def topological_hint(topo: dict, xy, yaw: float, goal: str) -> str | None:
    """One prompt line: the place route to `goal` and where the next opening is, relative to the robot."""
    from g1_factory.roadmap import compass_name, describe_direction
    from g1_factory.navigation import wrap_angle
    regions = {p["id"]: p["bounds"] for p in topo["places"] if p.get("bounds")}
    here = place_of(xy, regions, topo.get("aisle_segments", {}))
    if here == goal:
        return None
    path = route(topo, here, goal)
    if len(path) < 2:
        return None
    labels = {p["id"]: p["label"] for p in topo["places"]}
    door = next(e["door"] for e in topo["edges"] if {e["a"], e["b"]} == {path[0], path[1]})
    heading = math.atan2(door[1]-xy[1], door[0]-xy[0])
    bearing = math.degrees(wrap_angle(heading-yaw))
    distance = math.hypot(door[0]-xy[0], door[1]-xy[1])
    return (f"Topological route to {labels.get(goal, goal)}: " + " -> ".join(labels.get(p, p) for p in path)
            + f"; next opening (into {labels.get(path[1], path[1])}): {describe_direction(bearing)} "
              f"({bearing:+.0f} deg, {compass_name(heading)}), {distance:.1f} m")
