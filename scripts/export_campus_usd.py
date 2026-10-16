#!/usr/bin/env python3
"""Export the procedural Warehouse + Office campus (without the robot) to USD for Isaac Sim.

    python scripts/export_campus_usd.py --out outputs/isaac        # needs mujoco + usd-core (pxr)

Writes <out>/campus.usd (Z-up, metres; static colliders, PreviewSurface materials,
textured signs), <out>/textures/*.png and <out>/campus_meta.json (zones, obstacle
footprints and bounds used for the occupancy map and evaluation). Geometry comes
from the compiled MuJoCo scene, so both simulators see the same layout.
usd-core wheels exist for x86_64 Linux and macOS (Python <= 3.12); the ROS 2 Docker
image installs it on amd64.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory.scene import build_scene  # noqa: E402


def quat_wxyz(mat):
    """Rotation matrix -> unit quaternion (w, x, y, z)."""
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.asarray(mat, float).reshape(9))
    return q


def sanitize(name, used):
    base = re.sub(r"[^A-Za-z0-9_]", "_", name) or "geom"
    if base[0].isdigit():
        base = "g_"+base
    candidate, k = base, 1
    while candidate in used:
        k += 1
        candidate = f"{base}_{k}"
    used.add(candidate)
    return candidate


def material_textures(scene_xml: Path) -> dict:
    """MJCF material name -> texture PNG path (the compiled model keeps no file names)."""
    root = ET.parse(scene_xml).getroot()
    textures = {t.get("name"): t.get("file") for t in root.iter("texture") if t.get("file")}
    return {m.get("name"): textures.get(m.get("texture")) for m in root.iter("material") if m.get("texture")}


def main():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=ROOT/"outputs/isaac")
    parser.add_argument("--config", type=Path, default=ROOT/"configs/factory.yaml")
    parser.add_argument("--upstream", type=Path, default=ROOT/"external/unitree_rl_gym")
    args = parser.parse_args()
    out = args.out.resolve()
    (out/"textures").mkdir(parents=True, exist_ok=True)
    meta = build_scene(args.config, args.upstream, out/"mjcf"/"scene.xml")
    model = mujoco.MjModel.from_xml_path(meta["scene_path"])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    robot = np.zeros(model.nbody, bool)
    for b in range(model.nbody):
        p = b
        while p > 0:
            if p == pelvis:
                robot[b] = True
                break
            p = model.body_parentid[p]
    tex_of = material_textures(Path(meta["scene_path"]))

    usd_path = out/"campus.usd"
    if usd_path.exists():
        usd_path.unlink()
    stage = Usd.Stage.CreateNew(str(usd_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    campus = UsdGeom.Xform.Define(stage, "/Campus")
    stage.SetDefaultPrim(campus.GetPrim())
    UsdGeom.Scope.Define(stage, "/Campus/Looks")
    UsdGeom.Xform.Define(stage, "/Campus/Geoms")
    sun = UsdLux.DistantLight.Define(stage, "/Campus/Sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.XformCommonAPI(sun).SetRotate(Gf.Vec3f(35.0, -20.0, 0.0))
    sky = UsdLux.DomeLight.Define(stage, "/Campus/Sky")
    sky.CreateIntensityAttr(700.0)

    materials, used = {}, set()

    def material(rgba, texture=None):
        key = (tuple(np.round(rgba, 3)), texture)
        if key in materials:
            return materials[key]
        path = f"/Campus/Looks/m{len(materials)}"
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path+"/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(rgba[3]))
        if texture:
            reader = UsdShade.Shader.Define(stage, path+"/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            image = UsdShade.Shader.Define(stage, path+"/Texture")
            image.CreateIdAttr("UsdUVTexture")
            image.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{texture}")
            image.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            image.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(image.ConnectableAPI(), "rgb")
        else:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(float, rgba[:3])))
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[key] = mat
        return mat

    def place(prim, pos, rot, scale=None):
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        q = quat_wxyz(rot)
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        if scale is not None:
            xf.AddScaleOp().Set(Gf.Vec3f(*map(float, scale)))

    def quad(path, hx, hy, z=0.0):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([(-hx, -hy, z), (hx, -hy, z), (hx, hy, z), (-hx, hy, z)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([(0, 0, 1)]*4)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray([(0, 0), (1, 0), (1, 1), (0, 1)]))
        mesh.CreateDoubleSidedAttr(False)
        return mesh

    counts = {"box": 0, "textured": 0, "plane": 0, "cylinder": 0, "other": 0, "colliders": 0, "skipped": 0}
    for g in range(model.ngeom):
        if robot[model.geom_bodyid[g]]:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom_{g}"
        gtype, size = model.geom_type[g], model.geom_size[g]
        pos, rot = data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3)
        matid = model.geom_matid[g]
        mat_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MATERIAL, matid) if matid >= 0 else None
        texture_file = tex_of.get(mat_name) if mat_name else None
        rgba = model.mat_rgba[matid] if matid >= 0 else model.geom_rgba[g]
        visible = model.geom_group[g] != 3 and rgba[3] > 0
        collide = bool(model.geom_contype[g] or model.geom_conaffinity[g])
        texture = None
        if texture_file:
            texture = Path(texture_file).name
            shutil.copy(texture_file, out/"textures"/texture)
        path = "/Campus/Geoms/"+sanitize(name, used)
        if gtype == mujoco.mjtGeom.mjGEOM_BOX and texture and visible:
            prim = quad(path, size[0], size[1], size[2]+1e-3).GetPrim()  # texture on the +z face
            place(prim, pos, rot)
            counts["textured"] += 1
            if collide:
                cube = UsdGeom.Cube.Define(stage, path+"_collision")
                cube.CreateSizeAttr(2.0)
                place(cube.GetPrim(), pos, rot, size)
                UsdGeom.Imageable(cube).MakeInvisible()
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                counts["colliders"] += 1
        elif gtype == mujoco.mjtGeom.mjGEOM_BOX:
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(2.0)
            prim = cube.GetPrim()
            place(prim, pos, rot, size)
            counts["box"] += 1
        elif gtype == mujoco.mjtGeom.mjGEOM_PLANE:
            prim = quad(path, size[0], size[1]).GetPrim()
            place(prim, pos, rot)
            counts["plane"] += 1
            if collide:  # the floor: a 10 cm slab under the visual quad
                slab = UsdGeom.Cube.Define(stage, path+"_collision")
                slab.CreateSizeAttr(2.0)
                place(slab.GetPrim(), pos-rot[:, 2]*0.05, rot, (size[0], size[1], 0.05))
                UsdGeom.Imageable(slab).MakeInvisible()
                UsdPhysics.CollisionAPI.Apply(slab.GetPrim())
                counts["colliders"] += 1
            collide = False
        elif gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
            cyl = UsdGeom.Cylinder.Define(stage, path)
            cyl.CreateRadiusAttr(float(size[0]))
            cyl.CreateHeightAttr(float(2*size[1]))
            cyl.CreateAxisAttr(UsdGeom.Tokens.z)
            prim = cyl.GetPrim()
            place(prim, pos, rot)
            counts["cylinder"] += 1
        elif gtype == mujoco.mjtGeom.mjGEOM_SPHERE:
            sph = UsdGeom.Sphere.Define(stage, path)
            sph.CreateRadiusAttr(float(size[0]))
            prim = sph.GetPrim()
            place(prim, pos, rot)
            counts["other"] += 1
        elif gtype == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            sph = UsdGeom.Sphere.Define(stage, path)
            sph.CreateRadiusAttr(1.0)
            prim = sph.GetPrim()
            place(prim, pos, rot, size)
            counts["other"] += 1
        elif gtype == mujoco.mjtGeom.mjGEOM_CAPSULE:
            cap = UsdGeom.Capsule.Define(stage, path)
            cap.CreateRadiusAttr(float(size[0]))
            cap.CreateHeightAttr(float(2*size[1]))
            cap.CreateAxisAttr(UsdGeom.Tokens.z)
            prim = cap.GetPrim()
            place(prim, pos, rot)
            counts["other"] += 1
        else:
            counts["skipped"] += 1
            continue
        if visible:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material(rgba, texture))
        elif not texture:
            UsdGeom.Imageable(prim).MakeInvisible()
        if collide and gtype != mujoco.mjtGeom.mjGEOM_PLANE and not (texture and visible):
            UsdPhysics.CollisionAPI.Apply(prim)
            counts["colliders"] += 1
    stage.GetRootLayer().Save()

    zones = {zid: {k: z[k] for k in ("id", "label", "kind", "bounds", "center", "goal") if k in z}
             for zid, z in meta["zones"].items()}
    out_meta = {"source": "g1_factory.scene.build_scene", "bounds": meta["bounds"],
                "warehouse_bounds": meta["warehouse_bounds"], "office_bounds": meta["office_bounds"],
                "config": {"layout": meta["config"]["layout"]}, "zones": zones, "obstacles": meta["obstacles"],
                "camera": {"name": "head_rgbd", "pos_in_pelvis": [0.12, 0.0, 0.45], "pitch_down_deg": 10.0,
                           "fovy_deg": 54.9, "width": 640, "height": 480},
                "lidar": {"pos_in_pelvis": [0.05, 0.0, 0.42]}}
    (out/"campus_meta.json").write_text(json.dumps(out_meta, indent=1)+"\n")
    print(json.dumps({"usd": str(usd_path), "materials": len(materials), **counts}))


if __name__ == "__main__":
    main()
