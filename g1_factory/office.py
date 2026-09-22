"""Four furnished office rooms connected to the warehouse in one MJCF world.

All geometry and small signage textures are authored procedurally here. The
rooms are physically separated; furniture is static and collision-bearing.
Decoration and markings are visual only; glazed partitions have collisions.
Navigation uses the four-metre cross corridor and the clear room entrances.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


ROOMS = {
    "office_workstations": {
        "label": "Workstations", "bounds": [28., 2., 38., 12.],
        "center": [33., 7.], "goal": [34., 4.],
    },
    "office_meeting": {
        "label": "Meeting room", "bounds": [42., 2., 52., 12.],
        "center": [47., 7.], "goal": [46., 4.],
    },
    "office_lounge": {
        "label": "Staff lounge", "bounds": [28., -12., 38., -2.],
        "center": [33., -7.], "goal": [34., -4.],
    },
    "office_control": {
        "label": "Control room", "bounds": [42., -12., 52., -2.],
        "center": [47., -7.], "goal": [46., -4.],
    },
}


def _fmt(values) -> str:
    return " ".join(f"{float(v):.8g}" for v in values)


class _OfficeBuilder:
    """Small MJCF writer keeping planning footprints next to physical objects."""

    def __init__(self, world, asset, metadata: dict, output_dir: Path):
        self.world, self.asset, self.metadata = world, asset, metadata
        self.output_dir = output_dir
        self.wall = (0.80, 0.84, 0.86, 1)
        self.steel = (0.20, 0.27, 0.31, 1)
        self.wood = (0.63, 0.47, 0.31, 1)
        self.fabric = (0.20, 0.36, 0.40, 1)
        self.counter = 0

    def box(self, name, position, size, color, *, collision=True, material=None):
        attrs = {"name": f"factory_office_{name}", "type": "box",
                 "pos": _fmt(position), "size": _fmt([v / 2 for v in size]),
                 "rgba": _fmt(color), "contype": "1" if collision else "0",
                 "conaffinity": "1" if collision else "0"}
        if material:
            attrs["material"] = material
        return ET.SubElement(self.world, "geom", attrs)

    def cylinder(self, name, position, radius, height, color, *, collision=False):
        return ET.SubElement(self.world, "geom", name=f"factory_office_{name}",
                             type="cylinder", pos=_fmt(position),
                             size=_fmt([radius, height / 2]), rgba=_fmt(color),
                             contype="1" if collision else "0",
                             conaffinity="1" if collision else "0")

    def obstacle(self, name, x, y, sx, sy):
        self.metadata["obstacles"].append({
            "id": f"factory_office_{name}",
            "bounds": [x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2],
        })

    def wall_segment(self, name, x, y, sx, sy, *, glass=False):
        self.obstacle(name, x, y, sx, sy)
        if glass:
            # Low opaque base and a real, collision-bearing glazed partition.
            self.box(name + "_base", [x, y, .48], [sx, sy, .96], self.wall)
            self.box(name + "_glass", [x, y, 1.91], [sx, sy, 1.9],
                     (.47, .69, .74, .22))
            self.box(name + "_rail", [x, y, 2.92], [sx, sy, .16], self.steel)
            along_x = sx > sy
            length = sx if along_x else sy
            for index in range(max(1, int(length / 2.)) + 1):
                offset = -length / 2 + index * length / max(1, int(length / 2.))
                self.box(name + f"_mullion_{index}",
                         [x + offset if along_x else x,
                          y if along_x else y + offset, 1.93],
                         [.055, sy, 2.] if along_x else [sx, .055, 2.],
                         self.steel, collision=False)
        else:
            self.box(name, [x, y, 1.5], [sx, sy, 3.], self.wall)
            self.box(name + "_skirting", [x, y, .06],
                     [sx + .014, sy + .014, .12], self.steel, collision=False)

    def sign(self, name, title, subtitle, x, y, width=3.6):
        """Readable floor-mounted wayfinding; PNGs are generated, not external assets."""
        from PIL import Image, ImageDraw, ImageFont

        texture_dir = self.output_dir / "office_textures"
        texture_dir.mkdir(parents=True, exist_ok=True)
        path = texture_dir / f"{name}.png"
        image = Image.new("RGB", (1024, 256), (34, 53, 63))
        draw = ImageDraw.Draw(image)
        font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
        try:
            title_font = ImageFont.truetype(font_path, 68)
            subtitle_font = ImageFont.truetype(font_path, 32)
        except OSError:
            title_font = ImageFont.load_default(size=68)
            subtitle_font = ImageFont.load_default(size=32)
        draw.rounded_rectangle((3, 3, 1020, 252), 16, outline=(214, 183, 112), width=5)
        for text, font, top, color in ((title.upper(), title_font, 48, (248, 246, 235)),
                                       (subtitle.upper(), subtitle_font, 153, (201, 210, 214))):
            bbox = draw.textbbox((0, 0), text, font=font)
            draw.text(((1024 - bbox[2] + bbox[0]) / 2, top), text, font=font, fill=color)
        image.save(path)
        tex = f"office_texture_{name}"
        mat = f"office_material_{name}"
        ET.SubElement(self.asset, "texture", name=tex, type="2d", file=str(path.resolve()))
        ET.SubElement(self.asset, "material", name=mat, texture=tex,
                      texrepeat="1 1", texuniform="false", reflectance="0")
        # A finite plane maps one rectangular texture across its top surface.
        ET.SubElement(self.world, "geom", name=f"factory_office_sign_{name}",
                      type="plane", pos=_fmt([x, y, .012]),
                      size=_fmt([width / 2, width / 8, .001]), material=mat,
                      contype="0", conaffinity="0")

    def desk(self, name, x, y, width=2.15, depth=1.0, monitors=1):
        self.obstacle(name, x, y, width, depth)
        self.box(name + "_top", [x, y, .76], [width, depth, .08], self.wood)
        for side in (-1, 1):
            self.box(name + f"_leg_{side}", [x + side * (width / 2 - .14), y, .36],
                     [.09, depth - .12, .72], self.steel)
        for index in range(monitors):
            dx = (index - (monitors - 1) / 2) * .65
            self.monitor(name + f"_monitor_{index}", x + dx, y + depth * .25, .82)
        self.box(name + "_keyboard", [x, y - .14, .825], [.45, .16, .025],
                 (.19, .24, .26, 1), collision=False)
        self.box(name + "_mouse", [x + .4, y - .16, .83], [.07, .10, .04],
                 (.15, .20, .21, 1), collision=False)
        self.box(name + "_notebook", [x - width * .34, y - .16, .825],
                 [.26, .34, .025], (.88, .86, .78, 1), collision=False)

    def monitor(self, name, x, y, desk_height):
        self.box(name + "_stand", [x, y, desk_height + .11], [.07, .07, .22],
                 self.steel, collision=False)
        self.box(name + "_base", [x, y, desk_height + .015], [.28, .19, .035],
                 self.steel, collision=False)
        self.box(name + "_case", [x, y, desk_height + .38], [.59, .07, .37],
                 (.10, .15, .18, 1), collision=False)
        self.box(name + "_screen", [x, y - .037, desk_height + .38], [.535, .012, .31],
                 (.14, .39, .53, 1), collision=False)
        for i in range(3):
            self.box(name + f"_screen_line_{i}", [x - .025, y - .045, desk_height + .45 - i * .055],
                     [.34 - i * .04, .008, .012], (.50, .78, .84, 1), collision=False)

    def chair(self, name, x, y, yaw=0.):
        # Default chair faces +y. Rotate the offset of its back around its centre.
        self.obstacle(name, x, y, .68, .68)
        self.box(name + "_seat", [x, y, .47], [.58, .58, .11], self.fabric)
        self.cylinder(name + "_pedestal", [x, y, .25], .045, .40, self.steel,
                      collision=True)
        self.cylinder(name + "_base", [x, y, .06], .28, .06, self.steel)
        back = self.box(name + "_back", [x + math.sin(yaw) * .26,
                                        y - math.cos(yaw) * .26, .78],
                        [.58, .10, .53], self.fabric)
        back.set("quat", _fmt([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))
        for side in (-1, 1):
            arm = self.box(name + f"_arm_{side}",
                           [x + math.cos(yaw) * side * .28, y + math.sin(yaw) * side * .28, .66],
                           [.06, .43, .055], self.steel, collision=False)
            arm.set("quat", _fmt([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))

    def plant(self, name, x, y, height=1.4):
        self.obstacle(name, x, y, .65, .65)
        self.cylinder(name + "_pot", [x, y, .23], .28, .46,
                      (.48, .43, .37, 1), collision=True)
        self.cylinder(name + "_stem", [x, y, height / 2], .025, height,
                      (.36, .31, .21, 1))
        for i, (dx, dy, dz) in enumerate(((0, 0, 0), (.17, .1, -.12), (-.17, -.06, -.2))):
            ET.SubElement(self.world, "geom", name=f"factory_office_{name}_leaf_{i}",
                          type="ellipsoid", pos=_fmt([x + dx, y + dy, height + dz]),
                          size=".25 .24 .35", rgba=".25 .39 .29 1", contype="0", conaffinity="0")

    def sofa(self, name, x, y, width=2.8):
        self.obstacle(name, x, y, width, .98)
        self.box(name + "_base", [x, y, .32], [width, .92, .50], self.fabric)
        self.box(name + "_back", [x, y - .39, .75], [width, .18, .67], self.fabric)
        for side in (-1, 1):
            self.box(name + f"_arm_{side}", [x + side * (width / 2 - .1), y, .61],
                     [.20, .92, .4], self.fabric)
        for i in range(3):
            self.box(name + f"_cushion_{i}", [x + (i - 1) * (width - .5) / 3, y + .05, .61],
                     [(width - .6) / 3, .58, .16], (.35, .48, .48, 1), collision=False)


def add_office(world: ET.Element, asset: ET.Element, config: dict[str, Any],
               metadata: dict[str, Any], output_dir: str | Path) -> None:
    """Add a 24 × 24 m office east of the warehouse; mutate shared metadata.

    Fixed plan: bounds [28,-12,52,12], four rooms and a 4 m cross corridor.
    The west entrance at (28,0) joins the warehouse connector. Room goals are
    [34,4], [46,4], [34,-4], [46,-4]; door centres share those x coordinates.
    The y=0 spine and all door-to-goal paths have at least 0.6 m clearance.
    """
    metadata.setdefault("zones", {})
    metadata.setdefault("obstacles", [])
    metadata.setdefault("camera_names", {})
    b = _OfficeBuilder(world, asset, metadata, Path(output_dir).resolve())
    for room_id, defaults in ROOMS.items():
        room = metadata["zones"].setdefault(room_id, {})
        for key, value in defaults.items():
            room.setdefault(key, value)
        room.update({"id": room_id, "kind": "office"})
        room.setdefault("shelves", {})
        room.setdefault("sensors", [])

    # Floor finishes are paint only and do not add steps to G1's walking surface.
    b.box("corridor_floor", [40, 0, .002], [24, 24, .004], (.72, .76, .77, 1), collision=False)
    floors = [(0.61, .65, .63, 1), (.64, .65, .66, 1), (.64, .58, .49, 1), (.51, .59, .62, 1)]
    for index, (room_id, room) in enumerate(ROOMS.items()):
        xmin, ymin, xmax, ymax = room["bounds"]
        b.box(room_id + "_floor", [(xmin + xmax) / 2, (ymin + ymax) / 2, .006],
              [xmax - xmin, ymax - ymin, .004], floors[index], collision=False)
        # Tile joints add scale at overview and eye height without contact edges.
        for k in range(1, 10):
            b.box(room_id + f"_tile_x{k}", [xmin + k, (ymin + ymax) / 2, .009],
                  [.016, ymax - ymin, .002], (*floors[index][:3], .45), collision=False)
            b.box(room_id + f"_tile_y{k}", [(xmin + xmax) / 2, ymin + k, .009],
                  [xmax - xmin, .016, .002], (.45, .49, .49, .35), collision=False)

    # Exterior: the warehouse bridge enters through a four-metre west opening.
    for name, x, y, sx, sy in (("north_wall", 40, 12, 24.2, .2),
                               ("south_wall", 40, -12, 24.2, .2),
                               ("east_wall", 52, 0, .2, 24),
                               ("west_north_wall", 28, 7, .2, 10),
                               ("west_south_wall", 28, -7, .2, 10)):
        b.wall_segment(name, x, y, sx, sy, glass="west" in name or "east" in name)
    b.box("west_entrance_lintel", [28, 0, 2.79], [.2, 4, .42], b.wall)

    # Four bounded rooms. Split door walls leave 2.4 m openings into the spine.
    for room_id, room in ROOMS.items():
        xmin, ymin, xmax, ymax = room["bounds"]
        door_y = 2. if ymin > 0 else -2.
        door_x = room["goal"][0]
        inside_x = xmax if xmax == 38 else xmin
        b.wall_segment(room_id + "_side", inside_x, (ymin + ymax) / 2, .16, 10, glass=True)
        for side, lo, hi in (("left", xmin, door_x - 1.2), ("right", door_x + 1.2, xmax)):
            b.wall_segment(room_id + "_front_" + side, (lo + hi) / 2, door_y,
                           hi - lo, .16, glass=True)
        b.box(room_id + "_door_lintel", [door_x, door_y, 2.77], [2.4, .18, .46], b.steel)
        for direction in (-1, 1):
            b.box(room_id + f"_door_jamb_{direction}", [door_x + direction * 1.18, door_y, 1.27],
                  [.06, .21, 2.54], b.steel, collision=False)
        b.box(room_id + "_threshold_paint", [door_x, door_y, .013], [2.25, .12, .005],
              (.84, .66, .35, 1), collision=False)
        label_y = 3.0 if door_y > 0 else -3.0
        b.sign(room_id, room["label"], "OFFICE / " + f"0{list(ROOMS).index(room_id) + 1}",
               door_x, label_y)

    # Workstations: four full desks, monitors, keyboards and task chairs.
    for row, y in enumerate((8., 10.6)):
        for col, x in enumerate((30.6, 35.25)):
            name = f"workstation_{row}_{col}"
            b.desk(name, x, y)
            b.chair(name + "_chair", x, y - 1.04)
    b.plant("workstation_plant", 29.1, 5.7)
    b.box("workstation_bookshelf", [37.3, 9.2, .9], [.55, 2.5, 1.8], (.63, .52, .39, 1))
    b.obstacle("workstation_bookshelf", 37.3, 9.2, .55, 2.5)
    for i in range(6):
        b.box(f"workstation_books_{i}", [36.99, 8.2 + i * .36, 1.2], [.035, .2, .55],
              (.26 + (i % 3) * .13, .39, .42, 1), collision=False)

    # Meeting room: eight seats around a shared table and a wall display.
    b.obstacle("meeting_table", 46., 8.25, 4.4, 2.2)
    b.box("meeting_table_top", [46, 8.25, .78], [4.4, 2.2, .10], b.wood)
    for x in (44.7, 47.3):
        b.box(f"meeting_table_support_{x}", [x, 8.25, .37], [.24, 1.4, .74], b.steel)
    for i, x in enumerate((44.45, 46., 47.55)):
        b.chair(f"meeting_chair_south_{i}", x, 6.63)
        b.chair(f"meeting_chair_north_{i}", x, 9.88, math.pi)
    b.chair("meeting_chair_west", 43.1, 8.25, -math.pi / 2)
    b.chair("meeting_chair_east", 48.9, 8.25, math.pi / 2)
    for i, x in enumerate((44.6, 46., 47.4)):
        b.box(f"meeting_notepad_{i}", [x, 7.75, .84], [.25, .33, .02], (.90, .87, .79, 1), collision=False)
    b.box("meeting_display_frame", [46, 11.82, 1.95], [3.4, .15, 1.55], (.10, .16, .20, 1), collision=False)
    b.box("meeting_display", [46, 11.73, 1.95], [3.18, .02, 1.33], (.17, .40, .51, 1), collision=False)
    for i in range(4):
        b.box(f"meeting_display_line_{i}", [45.8, 11.71, 2.30 - .22 * i],
              [2.3 - i * .24, .015, .07], (.55, .75, .78, 1), collision=False)
    b.plant("meeting_plant", 50.7, 10.7, 1.6)

    # Lounge: upholstered seating, coffee table, refreshments and plants.
    b.sofa("lounge_sofa", 32.9, -10.3, 3.3)
    b.sofa("lounge_armchair", 36.2, -8.4, 1.25)
    b.obstacle("lounge_coffee_table", 32.9, -8.3, 2.3, 1.3)
    b.box("lounge_coffee_table", [32.9, -8.3, .40], [2.3, 1.3, .10], b.wood)
    for ix in (-1, 1):
        for iy in (-1, 1):
            b.box(f"lounge_table_leg_{ix}_{iy}", [32.9 + ix * .95, -8.3 + iy * .45, .2],
                  [.07, .07, .4], b.steel)
    for i in range(2):
        b.cylinder(f"lounge_cup_{i}", [32.4 + i * .85, -8.3, .5], .075, .11, (.9, .89, .83, 1))
    b.box("lounge_magazine", [33., -8., .465], [.28, .36, .02], (.40, .52, .54, 1), collision=False)
    b.obstacle("lounge_vending", 29.2, -6.2, 1.2, .9)
    b.box("lounge_vending", [29.2, -6.2, 1.], [1.2, .9, 2.], (.22, .29, .32, 1))
    b.box("lounge_vending_window", [29.2, -5.738, 1.24], [.86, .024, 1.1], (.33, .53, .58, 1), collision=False)
    for i in range(3):
        for j in range(3):
            b.box(f"lounge_vending_item_{i}_{j}", [28.94 + i * .25, -5.715, .92 + j * .29],
                  [.12, .01, .20], (.72, .58 + i * .06, .34 + j * .06, 1), collision=False)
    b.plant("lounge_plant", 29.4, -10.8, 1.7)
    b.plant("lounge_plant_small", 36.8, -10.9, 1.2)

    # Control room: multi-monitor consoles, overview displays, server cabinets.
    for i, x in enumerate((44.3, 47.25)):
        b.desk(f"control_console_{i}", x, -8.4, width=2.5, depth=1.1, monitors=3)
        b.chair(f"control_chair_{i}", x, -9.55)
    for i, x in enumerate((44.2, 47.45)):
        b.box(f"control_wall_display_{i}", [x, -11.8, 2.], [2.9, .16, 1.5],
              (.08, .14, .19, 1), collision=False)
        b.box(f"control_wall_screen_{i}", [x, -11.7, 2.], [2.65, .02, 1.25],
              (.12, .36, .46, 1), collision=False)
        for j in range(3):
            b.box(f"control_dashboard_panel_{i}_{j}", [x - .8 + j * .8, -11.68, 2.1],
                  [.63, .01, .8], (.25 + j * .08, .52, .54, 1), collision=False)
    for i, y in enumerate((-6.4, -8.4, -10.4)):
        b.obstacle(f"control_server_{i}", 50.8, y, 1.0, 1.25)
        b.box(f"control_server_{i}", [50.8, y, 1.15], [1., 1.25, 2.3], (.15, .20, .24, 1))
        for j in range(8):
            b.box(f"control_server_slot_{i}_{j}", [50.28, y, .35 + j * .235],
                  [.025, 1.02, .14], (.27, .33, .36, 1), collision=False)
            b.box(f"control_server_led_{i}_{j}", [50.26, y - .35, .35 + j * .235],
                  [.02, .035, .035], (.47, .78, .57, 1), collision=False)

    # Corridor wayfinding and overhead lighting remain collision-free.
    b.sign("office_entry", "OFFICE", "WORK / MEET / REST / CONTROL", 30.4, 0., width=3.4)
    for x in (34., 46.):
        for y in (-7., 7.):
            b.box(f"ceiling_light_{x}_{y}", [x, y, 3.35], [3.5, .32, .07],
                  (.95, .95, .87, 1), collision=False)
    ET.SubElement(world, "light", name="office_light", pos="40 0 12", dir="0 0 -1",
                  directional="true", diffuse="0.45 0.45 0.45", castshadow="false")
    # Camera's local +z points back from its target, local +y points upward.
    position, target = (56., -23., 30.), (40., 0., 0.)
    z = [position[i] - target[i] for i in range(3)]
    norm = math.sqrt(sum(v * v for v in z))
    z = [v / norm for v in z]
    x = [-z[1], z[0], 0.]
    norm = math.hypot(x[0], x[1])
    x = [v / norm for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    ET.SubElement(world, "camera", name="office_overview", pos=_fmt(position),
                  xyaxes=_fmt(x + y), fovy="52")
    ET.SubElement(world, "camera", name="office_map", pos="40 0 34", quat="1 0 0 0", fovy="45")
    metadata["camera_names"].update({"office": "office_overview", "office_map": "office_map"})
