#!/usr/bin/env python3
"""Figures of the semantic and topological map in the style of 3-D scene graph papers (Hydra, HOV-SG).

    bash scripts/g1_docker.sh python3 scripts/plot_scene_graph.py

Writes to --out (docs/images/semantic):
  scene_graph_3d.png           layered scene graph above a perspective render of the campus: scene geometry
                               (tinted by room), objects (by category), places and doorways (topological graph),
                               buildings, campus; thin vertical edges link every node to its parent,
  semantic_topological_2d.png  top-down render with rooms, objects, the place graph and the annotated anchors.
Input: outputs/isaac/semantic_map.json and campus_meta.json (scripts/export_campus_usd.py). The renders come from
the MuJoCo scene the Isaac USD is exported from (same geometry). --font / --font-bold choose the typeface.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory.semantic_world import CATEGORY_COLORS  # noqa: E402

SS = 2  # draw at 2x and downsample: anti-aliased lines, circles and text
FONTS = {False: "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
         True: "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"}
# Rooms: pastel (tinted floor, place nodes). Objects: saturated, by category.
ROOM_COLORS = [(141, 211, 199), (255, 237, 140), (190, 186, 218), (251, 128, 114), (128, 177, 211),
               (253, 180, 98), (179, 222, 105), (252, 205, 229), (188, 128, 189)]
OBJECT_COLORS = {"storage": (156, 107, 70), "production": (52, 101, 164), "energy": (222, 160, 20),
                 "pump": (0, 150, 170), "safety": (214, 39, 40), "office": (110, 110, 120)}
INK, BUILDING, ANCHOR = (34, 34, 34), (47, 72, 110), (46, 125, 50)
SHORT = {"Engineering workstations": "Workstations", "Warehouse main aisle": "Main aisle",
         "Warehouse-office bridge": "Bridge", "Pump room passage": "Pump passage"}
BUILDING_LABELS = {"warehouse": "Warehouse", "office": "Office", "utility": "Pump house"}
AISLE_BUILDING = {"warehouse_aisle": "warehouse", "office_corridor": "office", "pump_passage": "utility",
                  "bridge": "warehouse"}
HIDE = ("ceiling", "luminaire", "truss", "roof", "factory_floor", "blade")
CUTAWAY = ("office_south_wall", "office_south_door_lintel", "control_wall_display", "control_wall_screen",
           "utility_wall_south", "utility_wall_pipe")  # walls facing the 3-D camera
TOP_HIDE = ("lintel", "header", "pipe_support", "discharge_run", "room_sign", "corridor_sign", "zone_sign")


def font(size, bold=False):
    try:
        return ImageFont.truetype(FONTS[bold], int(size*SS))
    except OSError:
        return ImageFont.load_default()


def category_of(cls):
    for name, (_, classes) in CATEGORY_COLORS.items():
        if cls in classes:
            return name
    return "office"


# ------------------------------------------------------------------ rendering
class Scene:
    """The campus as MuJoCo renders it: robot hidden, one soft directional light plus the headlight."""

    def __init__(self):
        import mujoco
        from g1_factory.scene import build_scene
        meta = build_scene(ROOT/"configs/factory.yaml", ROOT/"external/unitree_rl_gym",
                           Path("/tmp/scene_graph/scene.xml"))
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(meta["scene_path"])
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
        self.groups = m.geom_group.copy()
        m.light_active[:] = 0
        m.light_active[0], m.light_type[0], m.light_castshadow[0] = 1, 1, 0  # 1 = directional
        m.light_dir[0] = np.array([0.35, 0.55, -0.76])
        m.light_diffuse[0], m.light_ambient[0], m.light_specular[0] = [0.55]*3, [0.15]*3, [0.0]*3
        m.vis.headlight.ambient[:], m.vis.headlight.diffuse[:], m.vis.headlight.specular[:] = 0.35, 0.3, 0.0
        mujoco.mj_forward(m, self.data)

    def render(self, size, lookat, distance, elevation, azimuth, fovy, hide):
        mujoco, m = self.mj, self.model
        W, H = size
        m.geom_group[:] = self.groups
        for g, name in enumerate(self.names):
            if not name or any(k in name for k in hide):  # unnamed geoms are the robot
                m.geom_group[g] = 5
        m.vis.global_.offwidth, m.vis.global_.offheight, m.vis.global_.fovy = W, H, fovy
        m.vis.map.znear, m.vis.map.zfar = 0.3*distance/m.stat.extent, 3*distance/m.stat.extent
        opt = mujoco.MjvOption()
        opt.geomgroup[5] = 0
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:], cam.distance, cam.elevation, cam.azimuth = lookat, distance, elevation, azimuth
        renderer = mujoco.Renderer(m, H, W)
        renderer.update_scene(self.data, cam, scene_option=opt)
        rgb = renderer.render().copy()
        renderer.enable_segmentation_rendering()
        renderer.update_scene(self.data, cam, scene_option=opt)
        background = renderer.render()[..., 0] < 0
        gl = renderer.scene.camera[0]
        proj = Projector(np.array(gl.pos), np.array(gl.forward), np.array(gl.up), fovy, W, H)
        renderer.close()
        rgb[background] = 255
        return Image.fromarray(rgb), proj


class Projector:
    """Pinhole projection of the render camera (vertical field of view fovy)."""

    def __init__(self, pos, forward, up, fovy, W, H):
        self.pos, self.fwd, self.up, self.right = pos, forward, up, np.cross(forward, up)
        self.f, self.c = (H/2)/math.tan(math.radians(fovy)/2), (W/2, H/2)

    def __call__(self, x, y, z=0.0):
        v = np.array([x, y, z], float)-self.pos
        depth = v @ self.fwd
        return self.c[0]+self.f*(v @ self.right)/depth, self.c[1]-self.f*(v @ self.up)/depth


# ------------------------------------------------------------------ graph
class Graph:
    """Nodes and parent links of the scene graph from the exported semantic map."""

    def __init__(self, semantic, meta):
        self.semantic = semantic
        self.places = {p["id"]: p for p in semantic["places"]}
        self.room_color = {z["id"]: ROOM_COLORS[i % len(ROOM_COLORS)] for i, z in enumerate(semantic["zones"])}
        kinds = {z["id"]: z.get("kind") for z in semantic["zones"]}
        bounds = {"warehouse": meta["warehouse_bounds"], "office": meta["office_bounds"]}
        if meta.get("utility_bounds"):
            u = meta["utility_bounds"]
            bounds["utility"] = [u[0], u[1], u[2], -16.0]
        self.buildings = {k: [(b[0]+b[2])/2, (b[1]+b[3])/2] for k, b in bounds.items()}
        self.place_building = {pid: kinds.get(pid) if kinds.get(pid) in bounds else AISLE_BUILDING.get(pid, "warehouse")
                               for pid in self.places}
        aisles = [pid for pid, p in self.places.items() if p["kind"] != "zone"]
        self.object_place = {}
        for o in semantic["objects"]:
            region = o.get("region")
            self.object_place[o["id"]] = region if region in self.places else min(
                aisles, key=lambda a: math.dist(o["center"][:2], self.places[a]["center"]))
        x0, y0, x1, y1 = semantic["occupancy"]["bounds"]
        self.campus = [(x0+x1)/2, (y0+y1)/2]

    def place_fill(self, pid):
        return self.room_color.get(pid, (255, 255, 255))


def dot(draw, xy, r, fill, outline=None, width=0):
    u, v = xy
    r, width = r*SS, width*SS
    draw.ellipse([u-r, v-r, u+r, v+r], fill=fill, outline=outline, width=int(round(width)))


def label(draw, xy, text, size, bold=False, fill=INK, anchor="md"):
    draw.text(xy, text, fill=fill, font=font(size, bold), anchor=anchor, stroke_width=int(3*SS),
              stroke_fill=(255, 255, 255))


def legend_bar(width, entries, size=22):
    """One row of legend entries: (text, kind, color) with kind dot / ring / door / square / tri."""
    probe = ImageDraw.Draw(Image.new("RGB", (1, 1)))
    f = font(size)
    total = sum(int(probe.textlength(t, font=f))+int(56*SS) for t, _, _ in entries)
    bar = Image.new("RGB", (max(width, total+int(40*SS)), int(56*SS)), (255, 255, 255))
    d = ImageDraw.Draw(bar)
    x, cy = (bar.width-total)//2, bar.height//2
    for text, kind, col in entries:
        c = (x+int(12*SS), cy)
        if kind == "dot":
            dot(d, c, 9, col, (255, 255, 255), 1.5)
        elif kind == "ring":
            dot(d, c, 10, col, INK, 2)
        elif kind == "door":
            dot(d, c, 6, (255, 255, 255), INK, 2)
        elif kind == "square":
            d.rounded_rectangle([c[0]-11*SS, cy-11*SS, c[0]+11*SS, cy+11*SS], radius=3*SS, fill=col)
        else:
            d.polygon([(c[0], cy-12*SS), (c[0]+11*SS, cy+9*SS), (c[0]-11*SS, cy+9*SS)], fill=col)
        d.text((x+int(30*SS), cy), text, fill=(60, 60, 60), font=f, anchor="lm")
        x += int(probe.textlength(text, font=f))+int(56*SS)
    return bar


def object_legend():
    return [(name, "dot", OBJECT_COLORS[name]) for name in OBJECT_COLORS]


def trim(img, pad):
    box = ImageChops.difference(img, Image.new("RGB", img.size, (255, 255, 255))).getbbox()
    return img.crop((max(box[0]-pad, 0), max(box[1]-pad, 0), min(box[2]+pad, img.width), min(box[3]+pad, img.height)))


# ------------------------------------------------------------------ 3-D scene graph
def scene_graph_3d(scene: Scene, graph: Graph, out: Path):
    sem = graph.semantic
    # Objects sit on the scene (node just above each object's box); the upper layers float clear of the back
    # walls even for the pump room at the front of the view.
    Z = {"places": 33.0, "buildings": 55.0, "campus": 63.0}
    img, P = scene.render((2200*SS, 2200*SS), lookat=(14.0, -5.0, 26.0), distance=200.0, elevation=-30.0,
                          azimuth=90.0, fovy=30.0, hide=HIDE+CUTAWAY)
    draw = ImageDraw.Draw(img, "RGBA")
    for z in sem["zones"]:  # rooms tint the geometry, as room segmentation does in Hydra figures
        x0, y0, x1, y1 = z["bounds"]
        draw.polygon([P(x0, y0, .05), P(x1, y0, .05), P(x1, y1, .05), P(x0, y1, .05)],
                     fill=(*graph.room_color[z["id"]], 110))

    place_xy = {pid: P(*p["center"], Z["places"]) for pid, p in graph.places.items()}
    building_xy = {k: P(*c, Z["buildings"]) for k, c in graph.buildings.items()}
    campus_xy = P(*graph.campus, Z["campus"])
    object_xy = {o["id"]: P(*o["center"][:2], o["aabb"][5]+0.4) for o in sem["objects"]}

    for o in sem["objects"]:
        draw.line([object_xy[o["id"]], place_xy[graph.object_place[o["id"]]]], fill=(120, 120, 120, 70), width=SS)
    for o in sem["objects"]:
        dot(draw, object_xy[o["id"]], 7, OBJECT_COLORS[category_of(o["class"])], (255, 255, 255), 2)
    for pid, b in graph.place_building.items():
        draw.line([place_xy[pid], building_xy[b]], fill=(80, 95, 120, 140), width=int(1.5*SS))
    for e in sem["edges"]:
        door = P(*e["door"], Z["places"])
        draw.line([place_xy[e["a"]], door, place_xy[e["b"]]], fill=(*INK, 235), width=4*SS, joint="curve")
    for e in sem["edges"]:
        dot(draw, P(*e["door"], Z["places"]), 5, (255, 255, 255), INK, 2)
    for pid, p in graph.places.items():
        dot(draw, place_xy[pid], 15 if p["kind"] == "zone" else 11, graph.place_fill(pid), INK, 2.5)
    for xy in building_xy.values():
        draw.line([xy, campus_xy], fill=(*INK, 190), width=int(2.5*SS))
    for xy in building_xy.values():
        u, v = xy
        draw.rounded_rectangle([u-15*SS, v-15*SS, u+15*SS, v+15*SS], radius=4*SS, fill=BUILDING,
                               outline=(255, 255, 255), width=2*SS)
    dot(draw, campus_xy, 18, INK, (255, 255, 255), 2.5)

    for pid, p in graph.places.items():
        u, v = place_xy[pid]
        label(draw, (u, v-(19 if p["kind"] == "zone" else 15)*SS), SHORT.get(p["label"], p["label"]), 21)
    for k, (u, v) in building_xy.items():
        label(draw, (u, v-21*SS), BUILDING_LABELS[k], 25, True, BUILDING)
    label(draw, (campus_xy[0], campus_xy[1]-24*SS), "Factory campus", 27, True)

    # Layer titles in a left column, at the mean height of each layer's nodes.
    heights = {"Campus": campus_xy[1], "Buildings": np.mean([v for _, v in building_xy.values()]),
               "Places": np.mean([v for _, v in place_xy.values()]), "Objects": P(*graph.campus, 0)[1]}
    details = {"Campus": "", "Buildings": f"{len(building_xy)} buildings",
               "Places": f"{len(place_xy)} places, {len(sem['edges'])} doorways",
               "Objects": f"{len(object_xy)} objects ({len({o['class'] for o in sem['objects']})} classes)\non the SLAM map"}
    box = ImageChops.difference(img, Image.new("RGB", img.size, (255, 255, 255))).getbbox()
    col = int(330*SS)
    fig = Image.new("RGB", (col+box[2]-box[0]+int(30*SS), box[3]-box[1]+int(60*SS)), (255, 255, 255))
    fig.paste(img.crop(box), (col, int(30*SS)))
    d = ImageDraw.Draw(fig)
    for name, v in heights.items():
        y = v-box[1]+30*SS
        d.text((col-30*SS, y), name, fill=INK, font=font(29, True), anchor="rs" if details[name] else "rm")
        if details[name]:
            d.multiline_text((col-30*SS, y+8*SS), details[name], fill=(110, 110, 110), font=font(20), anchor="ra",
                             align="right", spacing=4*SS)
        d.line([(col-18*SS, y), (col-4*SS, y)], fill=(170, 170, 170), width=2*SS)

    bar = legend_bar(fig.width, object_legend()+[("room", "ring", ROOM_COLORS[4]), ("corridor", "ring", (255, 255, 255)),
                                                  ("doorway", "door", INK), ("building", "square", BUILDING)])
    full = Image.new("RGB", (max(fig.width, bar.width), fig.height+bar.height), (255, 255, 255))
    full.paste(fig, (0, 0))
    full.paste(bar, ((full.width-bar.width)//2, fig.height))
    full = full.resize((full.width//SS, full.height//SS), Image.LANCZOS)
    trim(full, 16).save(out, optimize=True)
    return out


# ------------------------------------------------------------------ 2-D map
def semantic_topological_2d(scene: Scene, graph: Graph, out: Path, px_per_m=30):
    sem = graph.semantic
    x0, y0, x1, y1 = sem["occupancy"]["bounds"]
    s = px_per_m*SS
    W, H = int((x1-x0)*s), int((y1-y0)*s)
    fovy = 4.0  # far camera, narrow view: close to orthographic
    img, _ = scene.render((W, H), lookat=((x0+x1)/2, (y0+y1)/2, 0.0),
                          distance=((y1-y0)/2)/math.tan(math.radians(fovy/2)), elevation=-90.0, azimuth=90.0,
                          fovy=fovy, hide=HIDE+TOP_HIDE)
    img = Image.blend(img, Image.new("RGB", img.size, (255, 255, 255)), 0.25)
    px = lambda x, y: ((x-x0)*s, (y1-y)*s)
    draw = ImageDraw.Draw(img, "RGBA")
    for z in sem["zones"]:
        b, col = z["bounds"], graph.room_color[z["id"]]
        draw.rectangle([px(b[0], b[3]), px(b[2], b[1])], fill=(*col, 105),
                       outline=tuple(int(c*0.7) for c in col)+(230,), width=3*SS)
    centers = {pid: px(*p["center"]) for pid, p in graph.places.items()}
    for e in sem["edges"]:
        draw.line([centers[e["a"]], px(*e["door"]), centers[e["b"]]], fill=(*INK, 235), width=5*SS, joint="curve")
    for o in sem["objects"]:
        dot(draw, px(*o["center"][:2]), 7, OBJECT_COLORS[category_of(o["class"])], (255, 255, 255), 2)
    for a in sem["anchors"]:
        u, v = px(*a["xy"])
        draw.polygon([(u, v-13*SS), (u+12*SS, v+9*SS), (u-12*SS, v+9*SS)], fill=ANCHOR, outline=(255, 255, 255))
    for e in sem["edges"]:
        dot(draw, px(*e["door"]), 7, (255, 255, 255), INK, 3)
    for pid, p in graph.places.items():
        dot(draw, centers[pid], 16 if p["kind"] == "zone" else 12, graph.place_fill(pid), INK, 3)
    for z in sem["zones"]:
        b = z["bounds"]
        u, v = px(b[0], b[3])
        label(draw, (u+14*SS, v+12*SS), z["label"], 27, True, anchor="la")

    bar = legend_bar(W, object_legend()+[("room", "ring", ROOM_COLORS[4]), ("corridor", "ring", (255, 255, 255)),
                                         ("doorway", "door", INK), ("annotated anchor", "tri", ANCHOR)])
    full = Image.new("RGB", (W, H+bar.height), (255, 255, 255))
    full.paste(img, (0, 0))
    full.paste(bar, ((W-bar.width)//2, H))
    full.resize((W//SS, full.height//SS), Image.LANCZOS).save(out, optimize=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--semantic", type=Path, default=ROOT/"outputs/isaac/semantic_map.json")
    parser.add_argument("--meta", type=Path, default=ROOT/"outputs/isaac/campus_meta.json")
    parser.add_argument("--out", type=Path, default=ROOT/"docs/images/semantic")
    parser.add_argument("--font", default=FONTS[False])
    parser.add_argument("--font-bold", default=FONTS[True])
    args = parser.parse_args()
    FONTS[False], FONTS[True] = args.font, args.font_bold
    graph = Graph(json.loads(args.semantic.read_text()), json.loads(args.meta.read_text()))
    scene = Scene()
    args.out.mkdir(parents=True, exist_ok=True)
    a = scene_graph_3d(scene, graph, args.out/"scene_graph_3d.png")
    b = semantic_topological_2d(scene, graph, args.out/"semantic_topological_2d.png")
    print(json.dumps({"scene_graph_3d": str(a), "semantic_topological_2d": str(b)}))


if __name__ == "__main__":
    main()
