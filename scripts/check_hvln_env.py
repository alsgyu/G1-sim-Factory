#!/usr/bin/env python3
"""Pre-flight check for hierarchical-VLN experiments (run where the nodes will run).

    docker run --rm --gpus all --network host -v "$PWD":/ws g1-sim-ros2:humble \
        python3 scripts/check_hvln_env.py --vlm-model qwen3-vl:8b-instruct
    python3 scripts/check_hvln_env.py          # native, inside the sourced ROS 2 shell

Prints PASS / WARN / FAIL per item and exits 1 if anything FAILs.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
results = []


def report(level, item, detail):
    results.append(level)
    print(f"[{level}] {item}: {detail}", flush=True)


def check_ros():
    try:
        import rclpy  # noqa: F401
        report("PASS", "rclpy", f"ROS_DISTRO={os.environ.get('ROS_DISTRO', '?')}")
    except ImportError as exc:
        report("FAIL", "rclpy", f"{exc} - source /opt/ros/<distro>/setup.bash")
    # Isaac Sim's bundled ROS 2 uses Fast DDS: the whole graph must use it (scripts/g1_docker.sh sets both).
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    report("PASS" if rmw == "rmw_fastrtps_cpp" else "WARN", "RMW", rmw or "default (set rmw_fastrtps_cpp)")
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    report("PASS" if profile and Path(profile).is_file() else "WARN", "FASTRTPS_DEFAULT_PROFILES_FILE",
           profile or "unset (configs/fastdds.xml: UDP, 16 MB buffers)")
    try:
        rmem = int(Path("/proc/sys/net/core/rmem_max").read_text())
        report("PASS" if rmem >= 64*1024*1024 else "WARN", "net.core.rmem_max",
               f"{rmem} (>= 67108864 recommended for 0.9 MB images)")
    except OSError:
        report("WARN", "net.core.rmem_max", "not readable (non-Linux host)")


def check_assets() -> bool:
    upstream = ROOT/"external/unitree_rl_gym"
    if not (upstream/"resources/robots/g1_description/g1_12dof.xml").is_file():
        report("FAIL", "G1 assets", "missing - python3 scripts/fetch_assets.py")
        return False
    report("PASS", "G1 assets", str(upstream))
    return True


def check_sim(frames: int):
    """MuJoCo sensor rendering speed (development path only: --mujoco)."""
    upstream = ROOT/"external/unitree_rl_gym"
    os.environ.setdefault("MUJOCO_GL", "egl")
    import tempfile
    import mujoco
    import numpy as np
    from g1_factory.scene import build_scene
    from g1_factory.sensors import Mid360Lidar, RGBDCamera
    with tempfile.TemporaryDirectory() as tmp:
        meta = build_scene(ROOT/"configs/factory.yaml", upstream, Path(tmp)/"scene.xml")
        model = mujoco.MjModel.from_xml_path(meta["scene_path"])
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        camera = RGBDCamera(model, "head_rgbd", 640, 480, shadows=True)
        camera.render(data)
        begin = time.perf_counter()
        for _ in range(frames):
            rgb, depth = camera.render(data)
        ms = 1000*(time.perf_counter()-begin)/frames
        camera.close()
        level = "PASS" if ms < 15 else ("WARN" if ms < 33 else "FAIL")
        report(level, "RGB-D render 640x480 (shadows)",
               f"{ms:.1f} ms/frame with MUJOCO_GL={os.environ['MUJOCO_GL']} "
               f"({'GPU-class' if ms < 15 else 'too slow for 30 Hz real time; check --gpus all / NVIDIA EGL'})")
        if not (rgb.std() > 1 and (depth > 0).mean() > 0.2):
            report("FAIL", "RGB-D content", "blank image or no depth")
        lidar = Mid360Lidar(model)
        begin = time.perf_counter()
        xyz, *_ = lidar.scan(data)
        report("PASS" if time.perf_counter()-begin < 0.05 else "WARN", "MID-360 ray cast",
               f"{1000*(time.perf_counter()-begin):.1f} ms for {lidar.n} rays, {len(xyz)} returns")
        lidar.close()


def check_vlm(url: str, model: str):
    try:
        with urlopen(url.rstrip("/")+"/models", timeout=10) as response:
            models = [m.get("id") for m in json.load(response).get("data", [])]
    except Exception as exc:
        report("FAIL", "VLM server", f"{url}: {exc}")
        return
    report("PASS" if model in models else "FAIL", "VLM model", f"{model} {'available' if model in models else f'missing; have {models}'}")
    if model not in models:
        return
    import numpy as np
    from g1_factory.vlm import VLMClient
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[160:320, 220:420] = (220, 40, 40)
    client = VLMClient(url, model, timeout=300)
    call = client.chat("Reply with JSON only.", 'What colour is the square? {"colour": "..."}', [image], max_tokens=40)
    call = client.chat("Reply with JSON only.", 'What colour is the square? {"colour": "..."}', [image], max_tokens=40)
    report("PASS" if "red" in call.text.lower() else "WARN", "VLM image round trip",
           f"{call.latency_s:.2f} s warm, request {call.request_bytes} B, reply {call.text.strip()[:60]!r}")
    # The VLN's look-around sends four camera views at once (~1,050 tokens per image on Ollama).
    try:
        call = client.chat("Reply with JSON only.", 'How many images? {"count": ...}', [image]*4, max_tokens=20)
        report("PASS", "VLM four-image request (look-around)",
               f"{call.latency_s:.2f} s, {call.prompt_tokens or '?'} prompt tokens")
    except Exception as exc:
        report("FAIL", "VLM four-image request (look-around)",
               f"{exc}; Ollama: set OLLAMA_CONTEXT_LENGTH=8192 (docs/UBUNTU_ISAAC.md 1.4), vLLM: --max-model-len >= 8192")


def check_isaac():
    """Files the Isaac Sim 5.1 path needs (the container itself is checked by run_isaac_sim.sh --smoke-test)."""
    for path, how in ((ROOT/"outputs/isaac/campus.usd", "python3 scripts/export_campus_usd.py --out outputs/isaac"),
                      (ROOT/"outputs/isaac/campus_meta.json", "python3 scripts/export_campus_usd.py --out outputs/isaac"),
                      (ROOT/"external/unitree_rl_gym/resources/robots/g1_description/g1_12dof.urdf",
                       "python3 scripts/fetch_assets.py"),
                      (ROOT/"configs/fastdds.xml", "git pull"),
                      (ROOT/"outputs/isaac/semantic_map.json", "python3 scripts/export_campus_usd.py --out outputs/isaac")):
        report("PASS" if path.is_file() else "WARN", f"Isaac file {path.relative_to(ROOT)}",
               "present" if path.is_file() else f"missing (MuJoCo runs without it): {how}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vlm-url", default="http://127.0.0.1:11434/v1")
    parser.add_argument("--vlm-model", default="qwen3-vl:8b-instruct")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--skip-vlm", action="store_true")
    parser.add_argument("--mujoco", action="store_true", help="also time MuJoCo RGB-D/LiDAR rendering (dev path)")
    args = parser.parse_args()
    check_ros()
    if check_assets() and args.mujoco:
        check_sim(args.frames)
    check_isaac()
    if not args.skip_vlm:
        check_vlm(args.vlm_url, args.vlm_model)
    print(f"{results.count('PASS')} PASS, {results.count('WARN')} WARN, {results.count('FAIL')} FAIL")
    return 1 if "FAIL" in results else 0


if __name__ == "__main__":
    raise SystemExit(main())
