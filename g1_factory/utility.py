"""Pump room: a utility building south of the office, reached from the office's central corridor.

Plan (world metres): office south door x 38..42 at y=-12 -> 4 m passage (y -16..-12) -> pump room
[34, -26, 46, -16]. Equipment, modelled on a typical centrifugal pump station:

* skid P-101 / M-101 along x at y=-22: concrete plinth, steel base frame, pump volute, bearing
  housing, yellow coupling guard, finned electric motor with terminal box and fan cowl,
* suction line from the west wall with a gate valve, vertical discharge line with a check valve,
  red-wheel isolation valve and pressure gauge, header along the south wall,
* vertical pressure tank TK-101 with gauge, motor control centre MCC-1, parts rack with bins,
  yellow bollards, drain grating and walkway markings, name plates on everything inspectable.

Physical parts collide; small details are visual only. Footprints of parts that reach robot height
go to metadata["obstacles"] (occupancy map); walkable floor goes to metadata["free_areas"].
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOM = (34.0, -26.0, 46.0, -16.0)
PASSAGE = (38.0, -16.0, 42.0, -12.0)
BLUE, BLUE_DARK = (.11, .29, .66, 1), (.07, .18, .42, 1)
STEEL, STEEL_DARK = (.63, .65, .67, 1), (.36, .38, .40, 1)
CONCRETE, WALL = (.66, .66, .64, 1), (.74, .74, .72, 1)
YELLOW, RED, BLACK, WHITE = (.95, .76, .10, 1), (.78, .08, .06, 1), (.08, .08, .08, 1), (.94, .94, .92, 1)


def _fmt(values) -> str:
    return " ".join(f"{float(v):.8g}" for v in values)


class _Builder:
    def __init__(self, world, metadata):
        self.world, self.metadata = world, metadata

    def _geom(self, name, collide, **attrs):
        attrs.update(name=f"factory_utility_{name}", contype="1" if collide else "0",
                     conaffinity="1" if collide else "0")
        return ET.SubElement(self.world, "geom", {k: str(v) for k, v in attrs.items()})

    def obstacle(self, name, x0, y0, x1, y1):
        self.metadata["obstacles"].append({"id": f"factory_utility_{name}",
                                           "bounds": [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]})

    def box(self, name, center, size, rgba, collide=True, obstacle=None):
        """size = full extents. obstacle=True registers the footprint when it reaches robot height."""
        self._geom(name, collide, type="box", pos=_fmt(center), size=_fmt([s/2 for s in size]), rgba=_fmt(rgba))
        low, high = center[2]-size[2]/2, center[2]+size[2]/2
        if obstacle or (obstacle is None and collide and low < 1.8 and high > .08):
            self.obstacle(name, center[0]-size[0]/2, center[1]-size[1]/2, center[0]+size[0]/2, center[1]+size[1]/2)

    def pipe(self, name, a, b, radius, rgba, collide=False):
        self._geom(name, collide, type="cylinder", fromto=_fmt([*a, *b]), size=_fmt([radius]), rgba=_fmt(rgba))

    def ball(self, name, center, radius, rgba):
        self._geom(name, False, type="sphere", pos=_fmt(center), size=_fmt([radius]), rgba=_fmt(rgba))

    def ellipsoid(self, name, center, radii, rgba):
        self._geom(name, False, type="ellipsoid", pos=_fmt(center), size=_fmt(radii), rgba=_fmt(rgba))

    def valve(self, name, center, axis, wheel_up=.45, wheel_r=.2, body=.34):
        """Gate valve on a pipe along `axis` ("x"|"y"|"z") with a red handwheel on a stem."""
        x, y, z = center
        self.box(name+"_body", center, [body, body, body], BLUE_DARK, collide=False)
        for s in (-1, 1):  # flanges
            off = [0, 0, 0]
            off["xyz".index(axis)] = s*(body/2+.03)
            a = [center[i]+off[i]-(.03 if "xyz"[i] == axis else 0) for i in range(3)]
            b = [center[i]+off[i]+(.03 if "xyz"[i] == axis else 0) for i in range(3)]
            self.pipe(name+f"_flange_{s}", a, b, body*.75, STEEL_DARK)
        stem_dir = (0, 0, 1) if axis != "z" else (0, 1, 0)
        top = [x+stem_dir[0]*wheel_up, y+stem_dir[1]*wheel_up, z+stem_dir[2]*wheel_up]
        self.pipe(name+"_stem", center, top, .025, STEEL_DARK)
        a = [top[i]-stem_dir[i]*.02 for i in range(3)]
        b = [top[i]+stem_dir[i]*.02 for i in range(3)]
        self.pipe(name+"_handwheel", a, b, wheel_r, RED)

    def gauge(self, name, base, facing, stub=.2, radius=.12):
        """Pressure gauge on a short stub; dial faces `facing` (unit xy)."""
        fx, fy = facing
        end = [base[0]+fx*stub, base[1]+fy*stub, base[2]]
        self.pipe(name+"_stub", base, end, .03, STEEL_DARK)
        rim_back = [end[0]+fx*.02, end[1]+fy*.02, end[2]]
        rim_front = [end[0]+fx*.07, end[1]+fy*.07, end[2]]
        self.pipe(name+"_rim", rim_back, rim_front, radius, BLACK)
        face = [end[0]+fx*.075, end[1]+fy*.075, end[2]]
        self.pipe(name+"_dial", rim_front, face, radius*.86, WHITE)
        tip = [face[0]+fx*.004-fy*radius*.55, face[1]+fy*.004+fx*radius*.55, face[2]+radius*.35]
        self.pipe(name+"_needle", [face[0]+fx*.004, face[1]+fy*.004, face[2]], tip, .008, RED)


def add_pump_room(world: ET.Element, asset: ET.Element, config: dict, metadata: dict, output_dir: str | Path) -> None:
    from g1_factory.scene import _wall_sign  # textured plates, shared with the warehouse

    b = _Builder(world, metadata)
    out = Path(output_dir)
    x0, y0, x1, y1 = ROOM
    metadata.setdefault("free_areas", []).extend([[PASSAGE[0], PASSAGE[1]-.2, PASSAGE[2], PASSAGE[3]+.2],
                                                  list(ROOM)])
    metadata["utility_bounds"] = list(ROOM)

    # Floor paint (visual) and walkway markings.
    b.box("floor", [40, -21, .004], [12, 10, .006], CONCRETE, collide=False)
    b.box("passage_floor", [40, -14, .004], [4, 4, .006], (.70, .72, .72, 1), collide=False)
    for name, c, s in (("walkway_n", [40, -19.45, .009], [11.2, .09, .004]),
                       ("walkway_door_w", [38.55, -17.7, .009], [.09, 3.4, .004]),
                       ("walkway_door_e", [41.45, -17.7, .009], [.09, 3.4, .004]),
                       ("hatch_zone_w", [37.0, -22.0, .009], [.08, 2.6, .004]),
                       ("hatch_zone_e", [44.3, -22.0, .009], [.08, 2.6, .004])):
        b.box(name, c, s, YELLOW, collide=False)
    b.box("drain_grating", [40.6, -20.75, .008], [6.6, .28, .006], (.20, .21, .22, 1), collide=False)

    # Walls (concrete, 4 m), door openings in the north wall and the passage.
    walls = (("wall_south", [40, y0, 2], [12.2, .2, 4]), ("wall_west", [x0, -21, 2], [.2, 10, 4]),
             ("wall_east", [x1, -21, 2], [.2, 10, 4]), ("wall_north_w", [36, y1, 2], [4.2, .2, 4]),
             ("wall_north_e", [44, y1, 2], [4.2, .2, 4]), ("passage_wall_w", [38, -14, 1.5], [.2, 4, 3]),
             ("passage_wall_e", [42, -14, 1.5], [.2, 4, 3]))
    for name, c, s in walls:
        b.box(name, c, s, WALL)
    b.box("door_lintel", [40, y1, 3.55], [4, .2, .9], WALL, collide=False)
    for name, x, y in (("pillar_nw", x0+.3, y1-.3), ("pillar_ne", x1-.3, y1-.3),
                       ("pillar_sw", x0+.3, y0+.3), ("pillar_se", x1-.3, y0+.3)):
        b.box(name, [x, y, 2], [.6, .6, 4], (.70, .70, .68, 1))

    # Pump-motor skid P-101 / M-101 (axis along x at y=-22).
    sy, z = -22.0, .95
    b.box("skid_plinth", [40.5, sy, .08], [6.4, 2.0, .16], CONCRETE, obstacle=True)
    b.box("skid_frame", [40.5, sy, .25], [6.0, 1.5, .18], BLUE_DARK)
    b.pipe("pump_volute", [38.6, sy-.42, z], [38.6, sy+.42, z], .62, BLUE, collide=True)
    b.box("pump_volute_scroll", [38.6, sy, 1.5], [.55, .5, .5], BLUE, collide=False)
    b.box("pump_foot", [38.6, sy, .5], [.9, .9, .35], BLUE_DARK, collide=False)
    b.pipe("pump_bearing_housing", [39.2, sy, z], [39.95, sy, z], .22, BLUE, collide=True)
    b.box("coupling_guard", [40.4, sy, z], [.9, .58, .58], YELLOW)
    b.pipe("motor_body", [40.85, sy, z+.03], [42.75, sy, z+.03], .58, BLUE, collide=True)
    for i in range(7):
        x = 41.0+i*.27
        b.pipe(f"motor_fin_{i}", [x, sy, z+.03], [x+.05, sy, z+.03], .63, BLUE_DARK)
    b.pipe("motor_fan_cowl", [42.75, sy, z+.03], [43.2, sy, z+.03], .55, BLUE_DARK, collide=True)
    b.box("motor_terminal_box", [41.8, sy, 1.72], [.46, .38, .32], BLUE_DARK, collide=False)
    for x in (41.1, 42.5):
        b.box(f"motor_foot_{x:.1f}", [x, sy, .5], [.3, 1.1, .32], BLUE_DARK, collide=False)

    # Suction line from the west wall with a gate valve.
    b.pipe("suction_pipe", [x0+.15, sy, z], [38.0, sy, z], .2, STEEL, collide=True)
    b.obstacle("suction_pipe", x0+.1, sy-.25, 38.0, sy+.25)
    b.pipe("suction_flange", [37.94, sy, z], [38.02, sy, z], .3, STEEL_DARK)
    b.valve("suction_valve", [36.2, sy, z], "x", wheel_up=.55, wheel_r=.22)
    # Discharge: vertical riser with check valve, isolation valve and gauge, header along the south wall.
    b.pipe("discharge_riser", [38.6, sy, 1.75], [38.6, sy, 3.4], .17, STEEL)
    b.box("discharge_check_valve", [38.6, sy, 2.05], [.36, .36, .3], BLUE_DARK, collide=False)
    b.valve("discharge_valve", [38.6, sy, 2.75], "z", wheel_up=.42, wheel_r=.24)
    b.gauge("discharge_gauge", [38.6, sy+.17, 2.35], (0, 1))
    b.ball("discharge_elbow", [38.6, sy, 3.4], .2, STEEL)
    b.pipe("discharge_run", [38.6, sy, 3.4], [38.6, y0+.35, 3.4], .17, STEEL)
    b.ball("header_elbow", [38.6, y0+.35, 3.4], .2, STEEL)
    b.pipe("header", [x0+.4, y0+.35, 3.4], [x1-.4, y0+.35, 3.4], .17, STEEL)
    for i, x in enumerate((35.2, 42.6, 45.3)):
        b.pipe(f"wall_pipe_{i}", [x, y0+.3, 0], [x, y0+.3, 3.4], .07, STEEL)
    for i, x in enumerate((36.5, 44.0)):  # pipe supports
        b.box(f"pipe_support_{i}", [x, y0+.35, 3.12], [.12, .5, .12], STEEL_DARK, collide=False)

    # Pressure tank TK-101 (south-east) with legs, gauge and connection to the header.
    tx, ty = 44.4, -24.4
    for i, (dx, dy) in enumerate(((-.38, -.38), (.38, -.38), (-.38, .38), (.38, .38))):
        b.pipe(f"tank_leg_{i}", [tx+dx, ty+dy, 0], [tx+dx, ty+dy, .55], .045, STEEL_DARK)
    b.pipe("tank_shell", [tx, ty, .65], [tx, ty, 2.35], .58, (.80, .82, .84, 1), collide=True)
    b.obstacle("tank", tx-.65, ty-.65, tx+.65, ty+.65)
    b.ellipsoid("tank_head_top", [tx, ty, 2.35], [.58, .58, .28], (.80, .82, .84, 1))
    b.ellipsoid("tank_head_bottom", [tx, ty, .65], [.58, .58, .22], (.80, .82, .84, 1))
    b.gauge("tank_gauge", [tx+.25, ty+.53, 1.5], (0, 1), stub=.12)  # in view at stand-off distance
    b.pipe("tank_riser", [tx, ty, 2.6], [tx, ty, 3.4], .08, STEEL)
    b.valve("tank_valve", [tx-.95, ty+.0, 1.05], "x", wheel_up=.35, wheel_r=.16, body=.24)
    b.pipe("tank_inlet", [tx-1.5, ty, 1.05], [tx-.58, ty, 1.05], .08, STEEL)

    # Motor control centre MCC-1 (west wall) and a parts rack with bins (south-west).
    b.box("mcc_cabinet", [x0+.42, -18.6, 1.0], [.6, 1.6, 2.0], (.82, .84, .82, 1))
    for i, dy in enumerate((-.4, .4)):
        b.box(f"mcc_door_seam_{i}", [x0+.725, -18.6+dy, 1.0], [.01, .02, 1.8], STEEL_DARK, collide=False)
        b.box(f"mcc_lamp_{i}", [x0+.725, -18.6+dy, 1.75], [.012, .08, .08], (.10, .70, .20, 1) if i else RED,
              collide=False)
    b.box("rack_frame", [35.4, y0+.5, .9], [1.8, .6, 1.8], STEEL_DARK)
    bins = ((.12, .30, .60, 1), (.55, .56, .56, 1), (.12, .30, .60, 1))
    for level, zz in enumerate((.35, .95, 1.55)):
        b.box(f"rack_shelf_{level}", [35.4, y0+.5, zz-.12], [1.76, .56, .03], STEEL, collide=False)
        for k, xx in enumerate((34.85, 35.4, 35.95)):
            b.box(f"rack_bin_{level}_{k}", [xx, y0+.52, zz+.02], [.42, .42, .26], bins[(level+k) % 3], collide=False)

    # Bollards protecting the skid.
    for i, (x, y) in enumerate(((37.1, -20.8), (39.6, -20.8), (41.9, -20.8), (44.2, -20.8))):
        b.pipe(f"bollard_{i}", [x, y, 0], [x, y, 1.0], .08, YELLOW, collide=True)
        b.obstacle(f"bollard_{i}", x-.1, y-.1, x+.1, y+.1)

    # Name plates (camera-readable) and wayfinding.
    plates = (("pump_plate", "PUMP P-101", None, [38.6, sy+.76, .42], (0, 1), .78, .2, (.07, .18, .42)),
              ("motor_plate", "MOTOR M-101", None, [41.8, sy+.76, .42], (0, 1), .78, .2, (.07, .18, .42)),
              ("tank_plate", "TK-101", "PRESSURE TANK", [tx-.12, ty+.6, 1.0], (0, 1), .62, .3, (.30, .34, .38)),
              ("mcc_plate", "MCC-1", "MOTOR CONTROL", [x0+.735, -18.6, 1.45], (1, 0), .7, .3, (.30, .34, .38)),
              ("room_sign", "PUMP ROOM", "UTILITY / P-101", [40, y1+.12, 3.3], (0, 1), 2.6, .62, (.08, .34, .52)),
              ("room_sign_inside", "PUMP ROOM", "UTILITY", [x1-.12, -19.0, 2.6], (-1, 0), 2.4, .58, (.08, .34, .52)),
              ("corridor_sign", "PUMP ROOM", "SOUTH  /  UTILITY", [40, -11.9, 2.6], (0, 1), 2.4, .5, (.08, .34, .52)))
    for name, title, subtitle, pos, facing, w, h, color in plates:
        _wall_sign(asset, world, out, f"factory_utility_{name}", title, subtitle, pos, facing, w, h, color)
    ET.SubElement(world, "light", name="factory_light_utility", pos="40 -21 12", dir="0 0 -1",
                  directional="false", diffuse=".35 .35 .34", ambient=".02 .02 .02", castshadow="false")
