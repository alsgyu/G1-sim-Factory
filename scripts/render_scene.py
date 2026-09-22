"""Render real MuJoCo factory, selected bay, and ego RGB/depth previews."""
from pathlib import Path
import argparse
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from PIL import Image
from g1_factory.scene import build_scene
from g1_factory.locomotion import G1Locomotion

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/preview")
    parser.add_argument("--lab", default="lab_a")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    meta = build_scene(ROOT/"configs/factory.yaml", ROOT/"external/unitree_rl_gym", args.output/"scene.xml")
    model = mujoco.MjModel.from_xml_path(meta["scene_path"])
    data = mujoco.MjData(model)
    bay = meta["bays"][args.lab]
    policy = G1Locomotion(model, data, ROOT/"external/unitree_rl_gym")
    policy.reset(bay["spawn"][:2], bay["spawn"][2])
    for _ in range(500):
        policy.step([0, 0, 0])
    with mujoco.Renderer(model, height=800, width=1000) as renderer:
        renderer.update_scene(data, camera="map_camera")
        Image.fromarray(renderer.render()).save(args.output/"factory.png")
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [*bay["origin"], 0.7]
        camera.distance, camera.azimuth, camera.elevation = 12, 135, -55
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(args.output/"bay.png")
        renderer.update_scene(data, camera="ego_rgb")
        Image.fromarray(renderer.render()).save(args.output/"ego.png")
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        if not np.isfinite(depth).all() or np.min(depth) <= 0:
            raise RuntimeError("Depth image must be finite and positive in metres")
        np.save(args.output/"depth_m.npy", depth)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
