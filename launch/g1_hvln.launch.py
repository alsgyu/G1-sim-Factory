"""Hierarchical VLN stack on the MuJoCo G1 campus (ROS 2 Humble/Jazzy).

    ros2 launch launch/g1_hvln.launch.py task:=inspect_conveyor \
        vlm_url:=http://127.0.0.1:11434/v1 vlm_model:=qwen3-vl:8b-instruct seed:=0

Nodes: g1_sim, vln_agent, global_planner, path_follower, external_agent
(+ topic_monitor when monitor:=true). The graph stops when external_agent exits.
`seed` perturbs the spawn pose by up to spawn_noise_m / spawn_noise_deg so
repeated episodes are not identical. For Isaac Sim or a real G1, launch
everything except g1_sim (sim:=false) and remap topics (docs/HVLN.md).
"""
import json
import math
from pathlib import Path
import random

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]

DEFAULTS = {
    "task": ("inspect_conveyor", "Task id in tasks_file"),
    "tasks_file": ("", "Task YAML (default configs/vln_tasks.yaml)"),
    "planner_mode": ("vlm", "vlm: VLM plans subgoals; rule: alias-matching baseline"),
    "vlm_url": ("http://127.0.0.1:11434/v1", "OpenAI-compatible endpoint (Ollama, vLLM, ...)"),
    "vlm_model": ("qwen3-vl:8b-instruct", "Model name at vlm_url"),
    "condition": ("", "Label stored in summary.json for experiment tables"),
    "seed": ("0", "Spawn perturbation seed; 0 = exact task spawn"),
    "spawn_noise_m": ("0.3", "Max spawn position perturbation (m) when seed != 0"),
    "spawn_noise_deg": ("20.0", "Max spawn yaw perturbation (deg) when seed != 0"),
    "camera_rate_hz": ("30.0", "Head RGB-D rate (D435i default 30 Hz)"),
    "render_shadows": ("true", "Shadows in sensor images (cheap on a GPU, slow on CPU rendering)"),
    "realtime_factor": ("1.0", "Cap simulation speed; <= 0 runs as fast as possible"),
    "timeout_s": ("900.0", "Episode limit in simulation seconds"),
    "monitor": ("true", "Run topic_monitor (sizes, rates, QoS) - adds subscriber load"),
    "sim": ("true", "false: do not start g1_sim (Isaac Sim / real robot provides topics)"),
    "remaps": ("", "Comma-separated topic remaps applied to every node, e.g. /livox/lidar:=/point_cloud"),
    "output_dir": ("", "Episode output directory (default outputs/hvln_<task>)"),
}


def node(module, name, params, remaps=()):
    args = ["python3", "-m", f"g1_factory.ros2.{module}", "--ros-args", "-r", f"__node:={name}"]
    for remap in remaps:
        args += ["-r", remap]
    for key, value in params.items():
        if isinstance(value, (list, tuple)):
            value = "[" + ",".join(f"{float(v):.6f}" for v in value) + "]"
        elif isinstance(value, bool):
            value = str(value).lower()
        args += ["-p", f"{key}:={value}"]
    return ExecuteProcess(cmd=args, name=name, output="screen", cwd=str(ROOT))


def setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    task_name = arg("task")
    tasks_file = Path(arg("tasks_file") or ROOT/"configs/vln_tasks.yaml")
    task = yaml.safe_load(tasks_file.read_text(encoding="utf-8"))["tasks"][task_name]
    out = Path(arg("output_dir") or ROOT/"outputs"/f"hvln_{task_name}")
    out.mkdir(parents=True, exist_ok=True)
    seed = int(arg("seed"))
    x, y, yaw = (float(v) for v in task["spawn"])
    if seed:
        rng = random.Random(seed)
        radius = float(arg("spawn_noise_m"))*math.sqrt(rng.random())
        angle = rng.uniform(-math.pi, math.pi)
        x, y = x+radius*math.cos(angle), y+radius*math.sin(angle)
        yaw += math.radians(rng.uniform(-1, 1)*float(arg("spawn_noise_deg")))
    sim_time = {"use_sim_time": True}
    remaps = [r.strip() for r in arg("remaps").split(",") if r.strip()]
    if any(":=" not in r for r in remaps):
        raise ValueError("remaps must look like /from:=/to[,/from2:=/to2]")
    agent = node("external_agent_node", "external_agent",
                 {**sim_time, "task": task_name, "tasks_file": str(tasks_file), "output_dir": str(out),
                  "timeout_s": arg("timeout_s"), "condition": arg("condition") or arg("planner_mode"), "seed": seed},
                 remaps)
    actions = [
        node("vln_agent_node", "vln_agent", {**sim_time, "planner_mode": arg("planner_mode"),
                                             "vlm_url": arg("vlm_url"), "vlm_model": arg("vlm_model")}, remaps),
        node("planner_node", "global_planner", sim_time, remaps),
        node("follower_node", "path_follower", sim_time, remaps),
        agent,
        RegisterEventHandler(OnProcessExit(target_action=agent, on_exit=[EmitEvent(event=Shutdown(reason="task finished"))])),
    ]
    if arg("sim").lower() == "true":
        actions.insert(0, node("sim_node", "g1_sim", {
            "use_sim_time": False, "output_dir": str(out/"sim"), "spawn": [x, y, yaw],
            "camera_rate_hz": arg("camera_rate_hz"), "realtime_factor": arg("realtime_factor"),
            "render_shadows": arg("render_shadows").lower() == "true",
            # Quoted so ROS parses it as a YAML string, not a nested list.
            "unmapped_obstacles": "'" + json.dumps(task.get("unmapped_obstacles", [])) + "'"}))
    if arg("monitor").lower() == "true":
        actions.append(node("topic_monitor", "topic_monitor", {**sim_time, "output_dir": str(out)}, remaps))
    (out/"launch_args.json").write_text(json.dumps(
        {**{k: arg(k) for k in DEFAULTS}, "spawn_used": [x, y, yaw]}, indent=2)+"\n")
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()]
                             + [OpaqueFunction(function=setup)])
