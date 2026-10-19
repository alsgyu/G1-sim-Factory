#!/usr/bin/env python3
"""Paper-style figures of the semantic + topological map (layered 3-D scene graph, 2-D overlay).

    bash scripts/g1_docker.sh python3 scripts/plot_scene_graph.py --out docs/images/semantic

Writes
  scene_graph_3d.png  - layered scene graph in the style of Hydra / HOV-SG figures: campus geometry at the
                        bottom, then objects, places (topological graph with doorways), buildings, campus root,
                        with vertical edges from every node to its parent layer,
  semantic_topological_2d.png - top-down render of the campus with zones, objects and the place graph.
Input: outputs/isaac/semantic_map.json (scripts/export_campus_usd.py). The top-down geometry image is
rendered once from the scene source (outputs/isaac/floor_topdown.png, cached).
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

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
PLACE, DOOR, BUILDING, CAMPUS = (230, 90, 20), (255, 255, 255), (60, 90, 140), (40, 40, 40)
BUILDING_LABELS = {"warehouse": "Warehouse", "office": "Office", "utility": "Pump house"}
KEY_LABELS = {"PUMP P-101", "TK-101", "MCC-1", "STATION 2", "CHARGER 3", "SERVER RACK 3", "CONVEYOR A"}


def font(size, bold=False):
    try:
        return ImageFont.truetype(BOLD if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


def floor_render(bounds, path: Path, px_per_m=30):
    """Top-down near-orthographic render of the campus geometry (robot hidden, ceilings removed)."""
    if path.is_file():
        return Image.open(path).convert("RGB")
    import mujoco
    from g1_factory.scene import build_scene
    meta = build_scene(ROOT/"configs/factory.yaml", ROOT/"external/unitree_rl_gym", Path("/tmp/scene_graph/scene.xml"))
    model = mujoco.MjModel.from_xml_path(meta["scene_path"])
    data = mujoco.MjData(model)
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if any(k in name for k in ("ceiling", "luminaire", "truss", "lintel", "header", "pipe_support",
                                    "discharge_run", "room_sign", "corridor_sign", "zone_sign", "blade")):
            model.geom_rgba[gid, 3] = 0.0
    data.qpos[2] = -20.0  # robot out of view
    model.light_active[:] = 0  # flat, even shading from the headlight only (ceiling lights wash the floor out)
    model.vis.headlight.ambient[:] = 0.55
    model.vis.headlight.diffuse[:] = 0.45
    model.vis.headlight.specular[:] = 0.0
    model.vis.map.znear, model.vis.map.zfar = 1.0, 5000.0  # x extent; the camera is ~600 m away (near-orthographic)
    mujoco.mj_forward(model, data)
    x0, y0, x1, y1 = bounds
    w, h = int((x1-x0)*px_per_m), int((y1-y0)*px_per_m)
    model.vis.global_.offwidth, model.vis.global_.offheight = w, h
    model.vis.global_.fovy = 4.0
    renderer = mujoco.Renderer(model, h, w)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [(x0+x1)/2, (y0+y1)/2, 0.0]
    cam.distance = ((y1-y0)/2)/math.tan(math.radians(2.0))
    cam.elevation, cam.azimuth = -90.0, 90.0
    renderer.update_scene(data, cam)
    image = Image.fromarray(renderer.render())
    renderer.close()
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return image


def category_color(cls):
    for col, classes in CATEGORY_COLORS.values():
        if cls in classes:
            return col
    return (90, 90, 90)


def buildings(semantic, meta):
    """Building nodes from the aisle segments / zone kinds."""
    out = {}
    kinds = {z["id"]: z.get("kind") for z in semantic["zones"]}
    bounds = {"warehouse": meta["warehouse_bounds"], "office": meta["office_bounds"]}
    if meta.get("utility_bounds"):
        u = meta["utility_bounds"]
        bounds["utility"] = [u[0], u[1], u[2], -11.8]
    for key, b in bounds.items():
        out[key] = {"label": BUILDING_LABELS[key], "center": [(b[0]+b[2])/2, (b[1]+b[3])/2]}
    place_building = {}
    for p in semantic["places"]:
        kind = kinds.get(p["id"])
        place_building[p["id"]] = {"warehouse": "warehouse", "office": "office", "utility": "utility"}.get(
            kind, {"warehouse_aisle": "warehouse", "office_corridor": "office", "pump_passage": "utility",
                   "bridge": "warehouse"}.get(p["id"], "warehouse"))
    return out, place_building


class Projector:
    """Oblique view: azimuth about z, elevation, vertical exaggeration of layer heights."""

    def __init__(self, azimuth_deg=-12, elevation_deg=26, scale=20.0):
        self.a, self.e, self.s = math.radians(azimuth_deg), math.radians(elevation_deg), scale
        self.offset = (0.0, 0.0)

    def raw(self, x, y, z=0.0):
        u = x*math.cos(self.a)-y*math.sin(self.a)
        w = x*math.sin(self.a)+y*math.cos(self.a)
        return u*self.s, (-w*math.sin(self.e)-z*math.cos(self.e))*self.s

    def __call__(self, x, y, z=0.0):
        u, v = self.raw(x, y, z)
        return u-self.offset[0], v-self.offset[1]


def scene_graph_3d(semantic, meta, floor, out: Path):
    x0, y0, x1, y1 = semantic["occupancy"]["bounds"]
    layers = {"geometry": 0.0, "objects": 26.0, "places": 52.0, "buildings": 72.0, "campus": 86.0}
    names = {"geometry": "Geometry\n(SLAM map / scene)", "objects": f"Objects\n({len(semantic['objects'])}, semantic)",
             "places": f"Places + openings\n({len(semantic['places'])} / {len(semantic['edges'])}, topological)",
             "buildings": "Buildings", "campus": "Campus"}
    margin = 40+max(int(ImageDraw.Draw(Image.new("RGB", (1, 1))).multiline_textbbox((0, 0), t, font=font(28, True))[2])
                    for t in names.values())
    proj = Projector()
    corners = [proj.raw(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (0, layers["campus"]+4)]
    umin, vmin = min(c[0] for c in corners)-margin, min(c[1] for c in corners)-40
    umax, vmax = max(c[0] for c in corners)+40, max(c[1] for c in corners)+90
    proj.offset = (umin, vmin)
    W, H = int(umax-umin), int(vmax-vmin)
    img = Image.new("RGB", (W, H), (255, 255, 255))

    # Geometry layer: affine-warp the top-down render onto the ground plane.
    fw, fh = floor.size
    p00, p10, p01 = proj(x0, y1), proj(x1, y1), proj(x0, y0)  # image (0,0), (fw,0), (0,fh)
    A = np.array([[(p10[0]-p00[0])/fw, (p01[0]-p00[0])/fh], [(p10[1]-p00[1])/fw, (p01[1]-p00[1])/fh]])
    inv = np.linalg.inv(A)
    coeffs = (inv[0, 0], inv[0, 1], -inv[0, 0]*p00[0]-inv[0, 1]*p00[1],
              inv[1, 0], inv[1, 1], -inv[1, 0]*p00[0]-inv[1, 1]*p00[1])
    warped = floor.transform((W, H), Image.AFFINE, coeffs, resample=Image.BILINEAR, fillcolor=(255, 255, 255))
    mask = Image.new("L", floor.size, 255).transform((W, H), Image.AFFINE, coeffs, fillcolor=0)
    img.paste(warped, (0, 0), mask)
    draw = ImageDraw.Draw(img, "RGBA")

    def plane(z, fill, outline):
        pts = [proj(x0, y0, z), proj(x1, y0, z), proj(x1, y1, z), proj(x0, y1, z)]
        draw.polygon(pts, fill=fill, outline=outline)
        return pts

    for key in ("objects", "places"):
        plane(layers[key], (228, 236, 246, 90), (140, 160, 190, 200))
    for key, z in layers.items():
        u, v = proj(x0, y0, z)
        draw.multiline_text((u-24, v), names[key], fill=(40, 50, 70), anchor="rm", align="right",
                            font=font(28, True), spacing=4)

    objects = semantic["objects"]
    places = {p["id"]: p for p in semantic["places"]}
    bnodes, place_building = buildings(semantic, meta)
    place_xy = {pid: p["center"] for pid, p in places.items()}
    region_of = {o["id"]: (o["region"] if o["region"] in places else None) for o in objects}

    def nearest_aisle(xy):
        aisles = [pid for pid, p in places.items() if p["kind"] == "aisle"]
        return min(aisles, key=lambda a: math.dist(xy, place_xy[a]))

    # vertical edges first (under the nodes)
    for o in objects:
        x, y = o["center"][:2]
        parent = region_of[o["id"]] or nearest_aisle((x, y))
        draw.line([proj(x, y, 0.4), proj(x, y, layers["objects"])], fill=(120, 120, 120, 70), width=1)
        draw.line([proj(x, y, layers["objects"]), proj(*place_xy[parent], layers["places"])],
                  fill=(110, 120, 140, 60), width=1)
    for pid, b in place_building.items():
        draw.line([proj(*place_xy[pid], layers["places"]), proj(*bnodes[b]["center"], layers["buildings"])],
                  fill=(60, 90, 140, 110), width=2)
    campus_xy = ((x0+x1)/2, (y0+y1)/2)
    for b in bnodes.values():
        draw.line([proj(*b["center"], layers["buildings"]), proj(*campus_xy, layers["campus"])],
                  fill=(40, 40, 40, 150), width=3)

    for o in objects:  # object nodes
        u, v = proj(*o["center"][:2], layers["objects"])
        col = category_color(o["class"])
        draw.ellipse([u-6, v-6, u+6, v+6], fill=(*col, 255), outline=(255, 255, 255, 255), width=1)
        if o.get("label") in KEY_LABELS:
            draw.text((u+8, v-8), o["label"], fill=(30, 30, 30), font=font(16, True))
    for e in semantic["edges"]:  # topological edges through the opening
        a, b, d = place_xy[e["a"]], place_xy[e["b"]], e["door"]
        draw.line([proj(*a, layers["places"]), proj(*d, layers["places"]), proj(*b, layers["places"])],
                  fill=(*PLACE, 255), width=5)
    for e in semantic["edges"]:
        u, v = proj(*e["door"], layers["places"])
        draw.rectangle([u-6, v-6, u+6, v+6], fill=DOOR, outline=PLACE, width=3)
    for pid, p in places.items():
        u, v = proj(*p["center"], layers["places"])
        r = 13 if p["kind"] == "zone" else 10
        draw.ellipse([u-r, v-r, u+r, v+r], fill=PLACE if p["kind"] == "zone" else (255, 170, 80),
                     outline=(255, 255, 255), width=3)
        draw.text((u, v-r-4), p["label"], fill=(30, 40, 60), anchor="md", font=font(17, True))
    for b in bnodes.values():
        u, v = proj(*b["center"], layers["buildings"])
        draw.rounded_rectangle([u-16, v-16, u+16, v+16], radius=6, fill=BUILDING, outline=(255, 255, 255), width=3)
        draw.text((u, v-22), b["label"], fill=BUILDING, anchor="md", font=font(21, True))
    u, v = proj(*campus_xy, layers["campus"])
    draw.ellipse([u-18, v-18, u+18, v+18], fill=CAMPUS, outline=(255, 255, 255), width=3)
    draw.text((u, v-24), "G1 factory campus", fill=CAMPUS, anchor="md", font=font(23, True))

    lx, ly = 24, H-46
    for name, (col, _) in CATEGORY_COLORS.items():
        draw.ellipse([lx, ly, lx+18, ly+18], fill=col)
        draw.text((lx+26, ly-2), name, fill=(50, 50, 50), font=font(19))
        lx += 60+int(draw.textlength(name, font=font(19)))
    for name, col in (("zone place", PLACE), ("aisle place", (255, 170, 80)), ("opening", PLACE)):
        draw.ellipse([lx, ly, lx+18, ly+18], fill=col if name != "opening" else DOOR, outline=PLACE, width=3)
        draw.text((lx+26, ly-2), name, fill=(50, 50, 50), font=font(19))
        lx += 60+int(draw.textlength(name, font=font(19)))
    box = ImageChops.difference(img, Image.new("RGB", img.size, (255, 255, 255))).getbbox()
    img.crop((max(box[0]-30, 0), max(box[1]-30, 0), min(box[2]+30, W), min(box[3]+20, H))).save(out, optimize=True)
    return out


def overlay_2d(semantic, floor, out: Path):
    x0, y0, x1, y1 = semantic["occupancy"]["bounds"]
    img = floor.copy().convert("RGB")
    w, h = img.size
    sx, sy = w/(x1-x0), h/(y1-y0)
    px = lambda x, y: ((x-x0)*sx, (y1-y)*sy)
    draw = ImageDraw.Draw(img, "RGBA")
    palette = [(66, 133, 244), (52, 168, 83), (251, 188, 5), (234, 67, 53), (171, 71, 188), (0, 172, 193),
               (255, 112, 67), (124, 179, 66), (92, 107, 192)]
    zone_col = {z["id"]: palette[i % len(palette)] for i, z in enumerate(semantic["zones"])}
    for z in semantic["zones"]:
        b = z["bounds"]
        draw.rectangle([px(b[0], b[3]), px(b[2], b[1])], fill=(*zone_col[z["id"]], 38),
                       outline=(*zone_col[z["id"]], 235), width=5)
    places = {p["id"]: p["center"] for p in semantic["places"]}
    for e in semantic["edges"]:
        draw.line([px(*places[e["a"]]), px(*e["door"]), px(*places[e["b"]])], fill=(*PLACE, 255), width=7)
    for o in semantic["objects"]:
        u, v = px(*o["center"][:2])
        draw.ellipse([u-9, v-9, u+9, v+9], fill=(*category_color(o["class"]), 255), outline=(255, 255, 255), width=2)
    for e in semantic["edges"]:
        u, v = px(*e["door"])
        draw.rectangle([u-9, v-9, u+9, v+9], fill=DOOR, outline=PLACE, width=4)
    for p in semantic["places"]:
        u, v = px(*p["center"])
        r = 17 if p["kind"] == "zone" else 13
        draw.ellipse([u-r, v-r, u+r, v+r], fill=PLACE if p["kind"] == "zone" else (255, 170, 80),
                     outline=(255, 255, 255), width=4)
    for a in semantic["anchors"]:
        u, v = px(*a["xy"])
        draw.polygon([(u, v-16), (u+14, v+10), (u-14, v+10)], fill=(40, 150, 70), outline=(255, 255, 255))
    for z in semantic["zones"]:  # labels last, on top of everything
        b = z["bounds"]
        u, v = px(b[0], b[3])
        tw = draw.textlength(z["label"], font=font(26, True))
        draw.rounded_rectangle([u+6, v+6, u+18+tw, v+42], radius=6, fill=(*zone_col[z["id"]], 235))
        draw.text((u+12, v+9), z["label"], fill=(255, 255, 255), font=font(26, True))

    legend = [("place (zone)", "dot", PLACE), ("place (aisle)", "dot", (255, 170, 80)),
              ("opening / door", "square", DOOR), ("annotated anchor", "tri", (40, 150, 70))] + \
        [(name, "dot", col) for name, (col, _) in CATEGORY_COLORS.items()]
    bar = Image.new("RGB", (w, 54), (255, 255, 255))
    bd = ImageDraw.Draw(bar)
    lx = 16
    for text, shape, col in legend:
        if shape == "dot":
            bd.ellipse([lx, 16, lx+22, 38], fill=col)
        elif shape == "square":
            bd.rectangle([lx, 16, lx+22, 38], fill=col, outline=PLACE, width=4)
        else:
            bd.polygon([(lx+11, 14), (lx+23, 38), (lx-1, 38)], fill=col)
        bd.text((lx+30, 14), text, fill=(50, 50, 50), font=font(22))
        lx += 54+int(bd.textlength(text, font=font(22)))
    full = Image.new("RGB", (w, h+54), (255, 255, 255))
    full.paste(img, (0, 0))
    full.paste(bar, (0, h))
    full.save(out, optimize=True)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--semantic", type=Path, default=ROOT/"outputs/isaac/semantic_map.json")
    parser.add_argument("--meta", type=Path, default=ROOT/"outputs/isaac/campus_meta.json")
    parser.add_argument("--floor", type=Path, default=ROOT/"outputs/isaac/floor_topdown.png")
    parser.add_argument("--out", type=Path, default=ROOT/"docs/images/semantic")
    args = parser.parse_args()
    semantic = json.loads(args.semantic.read_text())
    meta = json.loads(args.meta.read_text())
    floor = floor_render(semantic["occupancy"]["bounds"], args.floor)
    args.out.mkdir(parents=True, exist_ok=True)
    a = scene_graph_3d(semantic, meta, floor, args.out/"scene_graph_3d.png")
    b = overlay_2d(semantic, floor, args.out/"semantic_topological_2d.png")
    print(json.dumps({"scene_graph_3d": str(a), "overlay_2d": str(b)}))


if __name__ == "__main__":
    main()
