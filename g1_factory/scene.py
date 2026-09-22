"""Procedural, editable four-bay factory for the upstream Unitree G1 model.

The factory geometry is created here; no proprietary warehouse assets are used.
Shelves are kinematic mocap bodies, so relocating them is an authoring operation,
not a claim that G1 can manipulate furniture. Temperatures are explicitly
synthetic observations of a Gaussian field, not a thermal simulation.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import re
from typing import Any
import xml.etree.ElementTree as ET

import yaml


def _number(value: Any, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"{label} must be finite and >= {minimum}")
    return value


def _vector(value: Any, n: int, label: str) -> list[float]:
    if not isinstance(value, (tuple, list)) or len(value) != n:
        raise ValueError(f"{label} must contain {n} numbers")
    return [_number(v, label) for v in value]


def _identifier(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", value):
        raise ValueError(f"{label} must start with a letter and contain only letters, digits or _")
    return value


def _shelf_half_extent(size: list[float], yaw: float) -> tuple[float, float]:
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return ((c * size[0] + s * size[1]) / 2,
            (s * size[0] + c * size[1]) / 2)


def _shelf_inside(position: list[float], size: list[float], yaw: float,
                  bay_size: list[float], wall_thickness: float, label: str) -> None:
    extent = _shelf_half_extent(size, yaw)
    for axis in (0, 1):
        if abs(position[axis]) + extent[axis] > (bay_size[axis] - wall_thickness) / 2:
            raise ValueError(f"{label} extends through a bay wall")


def _segment_hits_shelf(a: list[float], b: list[float], shelf: dict,
                        clearance: float) -> bool:
    """Segment/AABB intersection in shelf coordinates, expanded by clearance."""
    yaw = shelf.get("yaw", 0.0)
    c, s = math.cos(yaw), math.sin(yaw)
    def local(p):
        x, y = p[0] - shelf["position"][0], p[1] - shelf["position"][1]
        return (c * x + s * y, -s * x + c * y)
    p, q = local(a), local(b)
    t0, t1 = 0.0, 1.0
    for axis in (0, 1):
        bound = shelf["size"][axis] / 2 + clearance
        delta = q[axis] - p[axis]
        if abs(delta) < 1e-12:
            if abs(p[axis]) > bound:
                return False
        else:
            lo, hi = sorted(((-bound - p[axis]) / delta, (bound - p[axis]) / delta))
            t0, t1 = max(t0, lo), min(t1, hi)
            if t0 > t1:
                return False
    return True


def load_config(config_path: str | Path) -> dict:
    """Load YAML and reject invalid dimensions, bay assignments and unsafe routes."""
    with Path(config_path).expanduser().open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("Factory configuration must be a mapping")
    validate_config(config)
    return config


def validate_config(config: dict) -> None:
    if not isinstance(config, dict):
        raise ValueError("Factory configuration must be a mapping")
    layout = config.get("layout", {})
    if not isinstance(layout, dict):
        raise ValueError("layout must be a mapping")
    bay_size = _vector(layout.get("bay_size"), 2, "layout.bay_size")
    if min(bay_size) < 4:
        raise ValueError("Each bay dimension must be at least 4 metres")
    for key in ("corridor_width", "wall_thickness", "wall_height", "door_width",
                "floor_margin", "robot_clearance"):
        _number(layout.get(key), f"layout.{key}", 0.01)
    if layout["wall_thickness"] >= min(bay_size) / 4:
        raise ValueError("wall_thickness is too large for the bay")
    if not 1.2 <= layout["door_width"] < bay_size[0] - layout["wall_thickness"]:
        raise ValueError("door_width must be >= 1.2 m and smaller than the bay width")
    if layout["corridor_width"] <= 2 * layout["robot_clearance"]:
        raise ValueError("corridor_width must exceed twice robot_clearance")
    sim = config.get("simulation", {})
    if not isinstance(sim, dict):
        raise ValueError("simulation must be a mapping")
    dt = _number(sim.get("timestep", 0.002), "simulation.timestep", 0.00001)
    if dt > 0.01:
        raise ValueError("Use a simulation timestep <= 0.01 s for this G1 model")
    friction = _vector(sim.get("floor_friction", [1.0, 0.005, 0.0001]), 3,
                       "simulation.floor_friction")
    if min(friction) < 0 or friction[0] == 0:
        raise ValueError("Floor sliding friction must be positive; other friction >= 0")
    temperature = config.get("temperature", {})
    if not isinstance(temperature, dict):
        raise ValueError("temperature must be a mapping")
    _number(temperature.get("ambient_c", 23.0), "temperature.ambient_c")
    _number(temperature.get("temporal_amplitude_c", 0.0), "temperature.temporal_amplitude_c", 0)
    _number(temperature.get("period_s", 120.0), "temperature.period_s", 0.01)
    bays = config.get("bays", [])
    if not isinstance(bays, list) or len(bays) != 4:
        raise ValueError("Exactly four bays are required for the 2 x 2 factory")
    seen_ids, seen_cells = set(), set()
    for bay in bays:
        if not isinstance(bay, dict):
            raise ValueError("Each bay must be a mapping")
        bay_id = _identifier(bay.get("id"), "bay.id")
        if bay_id in seen_ids:
            raise ValueError(f"Duplicate bay id: {bay_id}")
        seen_ids.add(bay_id)
        grid = bay.get("grid")
        if not isinstance(grid, list) or len(grid) != 2 or any(type(i) is not int or i not in (0, 1) for i in grid):
            raise ValueError(f"{bay_id}.grid must be a unique [0|1, 0|1] cell")
        if tuple(grid) in seen_cells:
            raise ValueError(f"Duplicate factory grid cell: {grid}")
        seen_cells.add(tuple(grid))
        if not isinstance(bay.get("lab"), str) or not bay["lab"].strip():
            raise ValueError(f"{bay_id}.lab must be a nonempty laboratory name")
        spawn = _vector(bay.get("spawn"), 3, f"{bay_id}.spawn")
        waypoints = bay.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError(f"{bay_id}.waypoints must be a nonempty list")
        route = [spawn[:2]] + [_vector(p, 2, f"{bay_id}.waypoint") for p in waypoints]
        clearance = layout["robot_clearance"] + layout["wall_thickness"] / 2
        for p in route:
            if any(abs(p[i]) > bay_size[i] / 2 - clearance for i in (0, 1)):
                raise ValueError(f"{bay_id} spawn/route violates bay boundary clearance")
        shelf_ids = set()
        for collection in ("shelves", "sensors", "hotspots"):
            if not isinstance(bay.get(collection, []), list) or any(not isinstance(item, dict) for item in bay.get(collection, [])):
                raise ValueError(f"{bay_id}.{collection} must be a list of mappings")
        for shelf in bay.get("shelves", []):
            shelf_id = _identifier(shelf.get("id"), f"{bay_id}.shelf.id")
            if shelf_id in shelf_ids:
                raise ValueError(f"Duplicate shelf id in {bay_id}: {shelf_id}")
            shelf_ids.add(shelf_id)
            p = _vector(shelf.get("position"), 2, f"{bay_id}.{shelf_id}.position")
            size = _vector(shelf.get("size"), 3, f"{bay_id}.{shelf_id}.size")
            if min(size) < 0.12 or size[2] > layout["wall_height"]:
                raise ValueError(f"{bay_id}.{shelf_id} size must be >= 0.12 m and below wall height")
            yaw = _number(shelf.get("yaw", 0.0), "shelf.yaw")
            _shelf_inside(p, size, yaw, bay_size, layout["wall_thickness"], f"{bay_id}.{shelf_id}")
            if any(_segment_hits_shelf(a, b, shelf, layout["robot_clearance"])
                   for a, b in zip(route, route[1:])):
                raise ValueError(f"{bay_id} route intersects clearance around shelf {shelf_id}")
        sensor_ids = set()
        for sensor in bay.get("sensors", []):
            sid = _identifier(sensor.get("id"), f"{bay_id}.sensor.id")
            if sid in sensor_ids:
                raise ValueError(f"Duplicate sensor id in {bay_id}: {sid}")
            sensor_ids.add(sid)
            p = _vector(sensor.get("position"), 3, f"{bay_id}.{sid}.position")
            if any(abs(p[i]) >= bay_size[i] / 2 for i in (0, 1)) or not 0 < p[2] <= layout["wall_height"]:
                raise ValueError(f"{bay_id}.{sid} sensor must be inside its bay and above ground")
        for hotspot in bay.get("hotspots", []):
            p = _vector(hotspot.get("position"), 2, f"{bay_id}.hotspot.position")
            if any(abs(p[i]) > bay_size[i] / 2 for i in (0, 1)):
                raise ValueError(f"{bay_id} hotspot must be inside its bay")
            _number(hotspot.get("sigma_m"), "hotspot.sigma_m", 0.01)
            _number(hotspot.get("amplitude_c"), "hotspot.amplitude_c")


def _fmt(values) -> str:
    return " ".join(f"{float(v):.9g}" for v in values)


def _box(parent, name, position, halfsize, rgba, **extra):
    return ET.SubElement(parent, "geom", name=name, type="box", pos=_fmt(position),
                         size=_fmt(halfsize), rgba=_fmt(rgba),
                         contype="1", conaffinity="1", **extra)


def _absolute_assets(root: ET.Element, robot_file: Path) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler", angle="radian")
    assetdir = compiler.get("assetdir", "")
    for tag, directory in (("mesh", "meshdir"), ("texture", "texturedir")):
        asset_dir = (robot_file.parent / compiler.get(directory, assetdir)).resolve()
        for asset in root.findall(f".//asset/{tag}"):
            filename = asset.get("file")
            if filename:
                resolved = (asset_dir / filename).resolve()
                if not resolved.is_file():
                    raise FileNotFoundError(f"Missing upstream {tag}: {resolved}")
                asset.set("file", str(resolved))
        compiler.attrib.pop(directory, None)
    compiler.attrib.pop("assetdir", None)
    # Never let upstream strippath discard the absolute asset paths above.
    compiler.set("strippath", "false")


def build_scene(config_path: str | Path, upstream_root: str | Path,
                output_path: str | Path) -> dict:
    """Write MJCF and return serializable world-frame route/asset metadata.

    ``upstream_root`` is the checkout of unitree_rl_gym. All mesh references in
    the generated MJCF are absolute, so the output can be written anywhere.
    Cameras ``ego_rgb`` and ``map_camera`` can render either RGB or metric depth.
    The robot reset/spawn and policy initialization remain the runner's job.
    """
    config = load_config(config_path)
    upstream_root = Path(upstream_root).expanduser().resolve()
    robot_file = upstream_root / "resources/robots/g1_description/g1_12dof.xml"
    if not robot_file.is_file():
        raise FileNotFoundError(f"G1 model not found: {robot_file}; run python scripts/fetch_assets.py")
    root = ET.parse(robot_file).getroot()
    root.set("model", "G1_four_bay_factory")
    _absolute_assets(root, robot_file)
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", str(config.get("simulation", {}).get("timestep", 0.002)))
    option.set("gravity", "0 0 -9.81")
    world = root.find("worldbody")
    if world is None:
        raise ValueError("Upstream G1 XML has no worldbody")
    pelvis = world.find(".//body[@name='pelvis']")
    if pelvis is None:
        raise ValueError("Upstream G1 XML has no pelvis body")
    ET.SubElement(pelvis, "camera", name="ego_rgb", pos="0.15 0 0.50",
                  xyaxes="0 -1 0 0 0 1", fovy="75")
    layout = config["layout"]
    width, depth = layout["bay_size"]
    corridor, thickness, height = (layout[k] for k in ("corridor_width", "wall_thickness", "wall_height"))
    half_x = width + corridor / 2 + layout["floor_margin"]
    half_y = depth + corridor / 2 + layout["floor_margin"]
    for geom in list(world.findall("geom")):
        if geom.get("type") == "plane":
            world.remove(geom)
    ET.SubElement(world, "geom", name="factory_floor", type="plane", pos="0 0 0",
                  size=_fmt([half_x, half_y, 0.1]), rgba="0.73 0.75 0.76 1",
                  contype="1", conaffinity="1", condim="3",
                  friction=_fmt(config.get("simulation", {}).get("floor_friction", [1.0, 0.005, 0.0001])))
    ET.SubElement(world, "light", name="factory_light", pos="0 0 14", dir="0 0 -1",
                  directional="true", diffuse="0.8 0.8 0.8", castshadow="true")
    ET.SubElement(world, "camera", name="map_camera", pos=_fmt([0, 0, 2.7 * max(half_x, half_y)]),
                  quat="1 0 0 0", fovy="50")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "1280")
    global_visual.set("offheight", "960")
    bay_colors = [(0.35, 0.49, 0.61, 1), (0.46, 0.60, 0.45, 1),
                  (0.69, 0.52, 0.33, 1), (0.58, 0.45, 0.63, 1)]
    metadata = {"scene_path": str(Path(output_path).expanduser().resolve()),
                "config": copy.deepcopy(config), "bays": {},
                "camera_names": {"ego": "ego_rgb", "map": "map_camera"},
                "sensor_names": [], "synthetic_temperature": True,
                "bounds": [-half_x, -half_y, half_x, half_y]}
    for bay_index, bay in enumerate(config["bays"]):
        bay_id = bay["id"]
        gx, gy = bay["grid"]
        ox, oy = (gx - 0.5) * (width + corridor), (gy - 0.5) * (depth + corridor)
        def world_xy(p):
            return [ox + p[0], oy + p[1]]
        walls = ET.SubElement(world, "body", name=f"factory_{bay_id}_room", pos=_fmt([ox, oy, 0]))
        wall_color = (0.77, 0.79, 0.80, 1)
        for side, x in (("west", -width / 2), ("east", width / 2)):
            _box(walls, f"factory_{bay_id}_{side}_wall", [x, 0, height / 2],
                 [thickness / 2, depth / 2 + thickness / 2, height / 2], wall_color)
        door_side = 1 if gy == 0 else -1
        _box(walls, f"factory_{bay_id}_back_wall", [0, -door_side * depth / 2, height / 2],
             [width / 2, thickness / 2, height / 2], wall_color)
        segment_width = (width - layout["door_width"]) / 2
        for side in (-1, 1):
            _box(walls, f"factory_{bay_id}_door_wall_{'left' if side < 0 else 'right'}",
                 [side * (layout["door_width"] / 2 + segment_width / 2), door_side * depth / 2, height / 2],
                 [segment_width / 2, thickness / 2, height / 2], wall_color)
        # Painted strip is visual only, so it cannot catch the robot's feet.
        ET.SubElement(walls, "geom", name=f"factory_{bay_id}_door_marker", type="box",
                      pos=_fmt([0, door_side * (depth / 2 - 0.12), 0.002]),
                      size=_fmt([layout["door_width"] / 2, 0.08, 0.002]),
                      rgba=_fmt(bay_colors[bay_index]), contype="0", conaffinity="0")
        bay_meta = {"id": bay_id, "lab": bay["lab"], "origin": [ox, oy],
                    "spawn": world_xy(bay["spawn"]) + [bay["spawn"][2]],
                    "waypoints": [world_xy(p) for p in bay["waypoints"]],
                    "door": world_xy([0, door_side * depth / 2]),
                    "shelves": {}, "sensors": []}
        for shelf in bay.get("shelves", []):
            name = f"factory_shelf_{bay_id}_{shelf['id']}"
            yaw = shelf.get("yaw", 0.0)
            body = ET.SubElement(world, "body", name=name, mocap="true",
                                 pos=_fmt(world_xy(shelf["position"]) + [0]),
                                 quat=_fmt([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))
            sx, sy, sz = shelf["size"]
            for level in range(4):
                _box(body, f"{name}_board_{level}", [0, 0, 0.06 + level * (sz - 0.12) / 3],
                     [sx / 2, sy / 2, 0.025], (0.48, 0.51, 0.53, 1))
            for ix, x in enumerate((-sx / 2 + 0.035, sx / 2 - 0.035)):
                for iy, y in enumerate((-sy / 2 + 0.035, sy / 2 - 0.035)):
                    _box(body, f"{name}_post_{ix}_{iy}", [x, y, sz / 2],
                         [0.025, 0.025, sz / 2], bay_colors[bay_index])
            # Small collision-bearing stock boxes make racks visually legible.
            for index, x in enumerate((-sx / 4, sx / 4)):
                _box(body, f"{name}_stock_{index}", [x, 0, sz / 3 + 0.24],
                     [sx / 6, sy / 3, 0.13], (0.66, 0.54, 0.39, 1))
            bay_meta["shelves"][shelf["id"]] = {"body_name": name, **copy.deepcopy(shelf)}
        for sensor in bay.get("sensors", []):
            name = f"factory_sensor_{bay_id}_{sensor['id']}"
            position = world_xy(sensor["position"]) + [sensor["position"][2]]
            ET.SubElement(world, "site", name=name, type="box", pos=_fmt(position),
                          size="0.055 0.03 0.085", rgba="0.9 0.25 0.15 1")
            metadata["sensor_names"].append(name)
            bay_meta["sensors"].append({"id": sensor["id"], "site_name": name,
                                        "position": position, "local_position": sensor["position"]})
        metadata["bays"][bay_id] = bay_meta
    output = Path(metadata["scene_path"])
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return metadata


def move_shelf(model, data, metadata: dict, bay_id: str, shelf_id: str,
               local_xy, yaw: float = 0.0) -> None:
    """Relocate an existing rack (editor action); the robot cannot push a mocap body.

    Call while simulation is paused or while holding the viewer lock. The current
    route may become blocked after an edit: replanning is intentionally the caller's
    responsibility. Footprints crossing walls, other racks or G1 are rejected.
    """
    import mujoco
    position = _vector(local_xy, 2, "shelf.position")
    yaw = _number(yaw, "shelf.yaw")
    bay = metadata["bays"][bay_id]
    shelf = bay["shelves"][shelf_id]
    layout = metadata["config"]["layout"]
    _shelf_inside(position, shelf["size"], yaw, layout["bay_size"], layout["wall_thickness"], shelf_id)
    # Conservative oriented-rack bounds prevent intersecting geometry after edits.
    extent = _shelf_half_extent(shelf["size"], yaw)
    for other_id, other in bay["shelves"].items():
        if other_id == shelf_id:
            continue
        other_extent = _shelf_half_extent(other["size"], other.get("yaw", 0.0))
        if all(abs(position[i] - other["position"][i]) < extent[i] + other_extent[i] + 0.05 for i in (0, 1)):
            raise ValueError(f"Shelf placement overlaps {other_id}")
    ox, oy = bay["origin"]
    robot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    if robot_id >= 0:
        robot_local = [float(data.xpos[robot_id, 0]) - ox, float(data.xpos[robot_id, 1]) - oy]
        candidate = {"position": position, "size": shelf["size"], "yaw": yaw}
        if _segment_hits_shelf(robot_local, robot_local, candidate, layout["robot_clearance"]):
            raise ValueError("Shelf placement overlaps G1; move the robot first")
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, shelf["body_name"])
    if body_id < 0 or model.body_mocapid[body_id] < 0:
        raise ValueError(f"Missing mocap shelf body {shelf['body_name']}")
    mocap_id = model.body_mocapid[body_id]
    data.mocap_pos[mocap_id] = [ox + position[0], oy + position[1], 0]
    data.mocap_quat[mocap_id] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    shelf["position"], shelf["yaw"] = position, yaw
    mujoco.mj_forward(model, data)


def temperature_readings(metadata: dict, time_s: float = 0.0) -> list[dict]:
    """Return deterministic sensor readings in deg C from the configured mock field."""
    time_s = _number(time_s, "time_s", 0)
    cfg = metadata["config"]
    temp = cfg.get("temperature", {})
    ambient = temp.get("ambient_c", 23.0)
    fluctuation = temp.get("temporal_amplitude_c", 0.0) * math.sin(2 * math.pi * time_s / temp.get("period_s", 120.0))
    readings = []
    for bay_config in cfg["bays"]:
        bay_id = bay_config["id"]
        bay = metadata["bays"][bay_id]
        for sensor in bay["sensors"]:
            x, y = sensor["local_position"][:2]
            value = ambient + fluctuation
            for hotspot in bay_config.get("hotspots", []):
                distance2 = (x - hotspot["position"][0]) ** 2 + (y - hotspot["position"][1]) ** 2
                value += hotspot["amplitude_c"] * math.exp(-distance2 / (2 * hotspot["sigma_m"] ** 2))
            readings.append({"sensor_id": sensor["site_name"], "bay_id": bay_id,
                             "lab": bay["lab"], "position": sensor["position"],
                             "time_s": time_s, "temperature_c": float(value), "synthetic": True})
    return readings
