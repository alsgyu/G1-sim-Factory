"""Hierarchical VLN stack on the MuJoCo G1 campus (ROS 2 Humble/Jazzy).

    ros2 launch launch/g1_hvln.launch.py task:=inspect_conveyor \
        vlm_url:=http://127.0.0.1:11434/v1 vlm_model:=qwen3-vl:8b-instruct seed:=0

Nodes: g1_sim, vln_agent, vln_navigator (planner_mode:=vln) or global_planner
(oracle/rule baselines), path_follower, external_agent (+ topic_monitor when monitor:=true). The graph stops when external_agent exits.
With map_package:=maps/<name> also map_file; with package_frame:=map also localizer (TF map -> odom from LiDAR
scans and initial_pose:=x,y,yaw in the package's map frame, or RViz "2D Pose Estimate").
`seed` perturbs the spawn pose by up to spawn_noise_m / spawn_noise_deg so
repeated episodes are not identical. For Isaac Sim or a real G1, launch
everything except g1_sim (sim:=false) and remap topics (docs/HVLN.md).
"""
import json
import math
from pathlib import Path
import sys

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory.localization import compose, inverse  # noqa: E402
from g1_factory.map_package import world_from_map  # noqa: E402
from g1_factory.spawn import perturbed_spawn  # noqa: E402

DEFAULTS = {
    "task": ("inspect_conveyor", "Task id in tasks_file"),
    "tasks_file": ("", "Task YAML (default configs/vln_tasks.yaml)"),
    "planner_mode": ("vln", "vln: VLN navigator (vision+instruction picks waypoints); oracle/rule: map-lookup + A* baselines"),
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
    "sim": ("false", "default: Isaac Sim (scripts/run_isaac_sim.sh) or the real robot provides the topics; "
                     "true: start the MuJoCo g1_sim instead (development only)"),
    "use_sim_time": ("true", "Follow /clock (MuJoCo, Isaac Sim); false on a real robot"),
    "remaps": ("", "Comma-separated topic remaps applied to every node, e.g. /livox/lidar:=/point_cloud"),
    "agent": ("true", "false: no built-in external_agent/evaluator; send instructions yourself "
                      "(scripts/send_instruction.py) - the graph then runs until Ctrl-C"),
    "output_dir": ("", "Episode output directory (default outputs/hvln_<task>)"),
    "map_file": ("", "Run the VLN on a saved map (map_server YAML in the simulator frame, e.g. "
                     "maps/campus/map_world.yaml from g1_slam_mapping.launch.py) instead of the simulator's /map"),
    "map_package": ("", "Run the VLN on a map package (maps/<name>: the robot's SLAM map + a person's annotations, "
                        "scripts/build_map_package.py): occupancy, rooms, anchors and topology all from the package"),
    "package_frame": ("world", "world: package moved into the simulator frame with the known start pose (simulation); "
                               "map: the robot's own map frame, the localizer finds the robot in it (real robot)"),
    "initial_pose": ("", "package_frame:=map: rough pose x,y,yaw (m, rad) of the robot in the package's map frame "
                         "(within 1.5 m and 20 deg); empty: wait for /initialpose (RViz 2D Pose Estimate); "
                         "spawn (simulation): the task's spawn without the seed perturbation, moved into the map frame "
                         "with evaluation.json, as a person pointing at the start on the map"),
    "odom_drift": ("", "MuJoCo only: 'scale,yaw_deg_per_m' makes /odom drift like leg odometry (e.g. 0.05,1.0) to "
                       "check the localizer; the evaluator then reads the truth from /sim/ground_truth"),
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
    x, y, yaw = perturbed_spawn(task["spawn"], seed, float(arg("spawn_noise_m")), float(arg("spawn_noise_deg")))
    sim_time = {"use_sim_time": arg("use_sim_time").lower() == "true"}
    remaps = [r.strip() for r in arg("remaps").split(",") if r.strip()]
    if any(":=" not in r for r in remaps):
        raise ValueError("remaps must look like /from:=/to[,/from2:=/to2]")
    use_agent = arg("agent").lower() == "true"
    package = ({"map_package": str(Path(arg("map_package")).resolve()), "package_frame": arg("package_frame")}
               if arg("map_package") else {})
    localize = package.get("package_frame") == "map"
    if package and package["package_frame"] not in ("world", "map"):
        raise ValueError("package_frame must be world or map")
    agent = node("external_agent_node", "external_agent",
                 {**sim_time, "task": task_name, "tasks_file": str(tasks_file), "output_dir": str(out),
                  "timeout_s": arg("timeout_s"), "condition": arg("condition") or arg("planner_mode"), "seed": seed,
                  "truth_topic": "/sim/ground_truth" if arg("odom_drift") else "/odom", **package}, remaps)
    actions = [
        node("vln_agent_node", "vln_agent", {**sim_time, "planner_mode": arg("planner_mode"),
                                             "vlm_url": arg("vlm_url"), "vlm_model": arg("vlm_model"), **package},
             remaps),
        (node("vln_navigator_node", "vln_navigator", {**sim_time, "vlm_url": arg("vlm_url"),
                                                      "vlm_model": arg("vlm_model"), **package},
              remaps+(["/map:=/slam_map"] if arg("map_file") or arg("map_package") else []))
         if arg("planner_mode") == "vln" else node("planner_node", "global_planner", sim_time, remaps)),
        node("follower_node", "path_follower", sim_time, remaps),
    ]
    if use_agent:
        actions += [agent, RegisterEventHandler(OnProcessExit(
            target_action=agent, on_exit=[EmitEvent(event=Shutdown(reason="task finished"))]))]
    if arg("sim").lower() == "true":
        actions.insert(0, node("sim_node", "g1_sim", {
            "use_sim_time": False, "output_dir": str(out/"sim"), "spawn": [x, y, yaw],
            "camera_rate_hz": arg("camera_rate_hz"), "realtime_factor": arg("realtime_factor"),
            "render_shadows": arg("render_shadows").lower() == "true", "map_tf": not localize,
            "odom_drift": f"'{arg('odom_drift')}'",
            # Quoted so ROS parses it as a YAML string, not a nested list.
            "unmapped_obstacles": "'" + json.dumps(task.get("unmapped_obstacles", [])) + "'"}))
    if package:
        actions.append(node("map_file_node", "map_file", {**sim_time, "package": package["map_package"],
                                                          "frame": arg("package_frame"), "topic": "/slam_map"}))
    elif arg("map_file"):
        actions.append(node("map_file_node", "map_file", {**sim_time, "yaml": str(Path(arg("map_file")).resolve()),
                                                          "topic": "/slam_map"}))
    if localize:  # the robot finds itself in the package's map frame (no simulator start pose)
        initial = arg("initial_pose")
        if initial == "spawn":
            w = world_from_map(package["map_package"])
            initial = ",".join(f"{v:.3f}" for v in compose(inverse((w[0, 2], w[1, 2], math.atan2(w[1, 0], w[0, 0]))),
                                                           tuple(task["spawn"])))
        actions.append(node("localizer_node", "localizer", {**sim_time, "package": package["map_package"],
                                                             "initial_pose": f"'{initial}'"}, remaps))
    if arg("monitor").lower() == "true":
        actions.append(node("topic_monitor", "topic_monitor", {**sim_time, "output_dir": str(out)}, remaps))
    (out/"launch_args.json").write_text(json.dumps(
        {**{k: arg(k) for k in DEFAULTS}, "spawn_used": [x, y, yaw]}, indent=2)+"\n")
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()]
                             + [OpaqueFunction(function=setup)])
