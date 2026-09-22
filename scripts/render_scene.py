"""Render actual MuJoCo Warehouse + Office RGB/depth previews, without a GUI.

Run on Ubuntu with ``MUJOCO_GL=egl python scripts/render_scene.py``. Preview
images use the same compiled scene and released walking policy as the runner.
No illustrative/generated image assets are substituted for simulation output.
"""
from pathlib import Path
import argparse
import json
import os
import sys

# Keep an explicitly selected backend (e.g. osmesa); EGL needs no X desktop.
os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from PIL import Image
from g1_factory.scene import build_scene
from g1_factory.locomotion import G1Locomotion
from g1_factory.routes import resolve_scenario

ROOT = Path(__file__).resolve().parents[1]


def overview_camera(bounds, *, elevation=-59, azimuth=90, distance_factor=0.92):
    """Cutaway perspective with enough margin for the full physical footprint."""
    xmin, ymin, xmax, ymax = bounds
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [(xmin + xmax) / 2, (ymin + ymax) / 2, 1.0]
    camera.distance = max(xmax - xmin, ymax - ymin) * distance_factor
    camera.azimuth, camera.elevation = azimuth, elevation
    return camera


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/preview")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/factory.yaml")
    parser.add_argument("--upstream", type=Path, default=ROOT / "external/unitree_rl_gym")
    parser.add_argument("--scenario", default="warehouse_tour")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1000)
    args = parser.parse_args(argv)
    if not 320 <= args.width <= 4096 or not 240 <= args.height <= 4096:
        parser.error("Use 320..4096 px width and 240..4096 px height")
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = build_scene(args.config, args.upstream, args.output / "scene.xml")
    scenario = resolve_scenario(metadata, name=args.scenario)
    model = mujoco.MjModel.from_xml_path(metadata["scene_path"])
    # Offscreen size is independent of the desktop viewer's window size.
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    walking = G1Locomotion(model, data, args.upstream)
    walking.reset(scenario["spawn"][:2], scenario["spawn"][2])
    for _ in range(round(1.0 / model.opt.timestep)):
        walking.step([0, 0, 0])
    if not np.isfinite(data.qpos).all() or data.qpos[2] < 0.45:
        raise RuntimeError("G1 did not remain upright during preview settling")
    cameras = metadata["camera_names"]
    follow = mujoco.MjvCamera()
    follow.type = mujoco.mjtCamera.mjCAMERA_FREE
    follow.lookat[:] = [float(data.qpos[0]), float(data.qpos[1]), 0.9]
    follow.distance, follow.azimuth, follow.elevation = 4.8, 110, -14
    views = {
        "campus": overview_camera(metadata["bounds"]),
        "warehouse": overview_camera(metadata["warehouse_bounds"], elevation=-54, azimuth=125, distance_factor=1.45),
        "office": overview_camera(metadata["office_bounds"], elevation=-58, azimuth=125, distance_factor=1.95),
        "g1": follow,
        "ego": cameras["ego"],
    }
    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        # Ceilings, if enabled in a custom config, can be hidden through group 4
        # without changing collisions or the world rendered by the actual runner.
        options = mujoco.MjvOption()
        options.geomgroup[4] = 0
        for name, camera in views.items():
            # Ego must retain the same visible geometry as policy input.
            renderer.update_scene(data, camera=camera,
                                  scene_option=None if name == "ego" else options)
            rgb = renderer.render().copy()
            Image.fromarray(rgb).save(args.output / f"{name}.png")
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        if not np.isfinite(depth).all() or np.min(depth) <= 0:
            raise RuntimeError("Depth image must be finite and positive in metres")
        np.save(args.output / "depth_m.npy", depth)
    manifest = {
        "source": "MuJoCo RGB render; physical scene, no illustrative replacements",
        "mujoco": mujoco.__version__, "scenario": scenario["name"],
        "spawn_world_xy_yaw": scenario["spawn"], "simulation_time_s": float(data.time),
        "width": args.width, "height": args.height,
        "views": list(views), "depth": "ego metric metres; depth_m.npy",
    }
    (args.output / "preview.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
