"""Localisation measurement: walk a route and record the localizer against the simulator's truth (docs/METRICS.md).

    # terminal 1 (Isaac Sim at the route's start, the simulator keeps its own map -> odom)
    bash scripts/run_isaac_sim.sh --spawn 12 -3.8 1.5708
    # terminal 2
    bash scripts/g1_docker.sh ros2 launch launch/g1_localization_eval.launch.py map_package:=maps/campus

Nodes: localizer on the map package (publish_tf:=false: it only measures), mapping_tour (wait_for_slam:=false)
with global_planner and path_follower walking the route of configs/metrics.yaml on the simulator's map, and
localization_eval recording samples.csv, fix_trials.csv, status.jsonl and run.json in output_dir (+ g1_sim with
sim:=true, the MuJoCo development graph). The launch ends when the route is done.
The localizer starts from a rough pose: the route's start moved into the package's map frame (evaluation.json)
plus initial_offset, as a person pointing at the start on the map.
Then: bash scripts/g1_docker.sh python3 scripts/eval_localization.py <output_dir>
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

DEFAULTS = {
    "map_package": ("", "Map package (maps/<name>) with evaluation.json (made in simulation)"),
    "tour": ("localization", "Route in tour_file"),
    "tour_file": (str(ROOT/"configs/metrics.yaml"), "Routes (spawn + waypoints, simulator frame)"),
    "initial_offset": ("0.8,-0.6,12", "Error of the initial pose given to the localizer: dx, dy (m), dyaw (deg)"),
    "trial_every_m": ("3.0", "Walking distance between first-fix trials"),
    "output_dir": (str(ROOT/"outputs/metrics/localization"), "Where the records go"),
    "truth_semantic": ("", "Simulator semantic map (world frame); default outputs/isaac/semantic_map.json"),
    "sim": ("false", "true: start the MuJoCo g1_sim (development); default: Isaac Sim provides the topics"),
    "camera_rate_hz": ("2.0", "MuJoCo only: the camera is not needed here"),
    "realtime_factor": ("1.0", "MuJoCo only"),
}


def node(module, name, params):
    args = ["python3", "-m", f"g1_factory.ros2.{module}", "--ros-args", "-r", f"__node:={name}"]
    for key, value in params.items():
        if isinstance(value, (list, tuple)):
            value = "[" + ",".join(f"{float(v):.6f}" for v in value) + "]"
        elif isinstance(value, bool):
            value = str(value).lower()
        args += ["-p", f"{key}:={value}"]
    return ExecuteProcess(cmd=args, name=name, output="screen", cwd=str(ROOT))


def setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    if not arg("map_package"):
        raise ValueError("map_package:=maps/<name> is required")
    package = Path(arg("map_package")).resolve()
    tour = yaml.safe_load(Path(arg("tour_file")).read_text())["tours"][arg("tour")]
    W = world_from_map(package)
    map_from_world = inverse((W[0, 2], W[1, 2], math.atan2(W[1, 0], W[0, 0])))
    dx, dy, ddeg = (float(v) for v in arg("initial_offset").split(","))
    start = compose(map_from_world, tuple(tour["spawn"]))
    initial = (start[0]+dx, start[1]+dy, start[2]+math.radians(ddeg))
    out = Path(arg("output_dir"))
    out.mkdir(parents=True, exist_ok=True)
    sim_time = {"use_sim_time": True}
    evaluator = node("localization_eval_node", "localization_eval", {
        **sim_time, "package": str(package), "output_dir": str(out), "trial_every_m": arg("trial_every_m"),
        **({"truth_semantic": arg("truth_semantic")} if arg("truth_semantic") else {})})
    actions = [
        node("localizer_node", "localizer", {**sim_time, "package": str(package), "publish_tf": False,
                                             "map_frame": "package_map",
                                             "initial_pose": "'" + ",".join(f"{v:.3f}" for v in initial) + "'"}),
        node("planner_node", "global_planner", sim_time),
        node("follower_node", "path_follower", sim_time),
        node("mapping_tour_node", "route", {**sim_time, "tour_file": arg("tour_file"), "tour": arg("tour"),
                                            "wait_for_slam": False}),
        evaluator,
        RegisterEventHandler(OnProcessExit(target_action=evaluator,
                                           on_exit=[EmitEvent(event=Shutdown(reason="route measured"))])),
    ]
    if arg("sim").lower() == "true":
        actions.insert(0, node("sim_node", "g1_sim", {
            "use_sim_time": False, "output_dir": str(out/"sim"), "spawn": tour["spawn"],
            "camera_rate_hz": arg("camera_rate_hz"), "realtime_factor": arg("realtime_factor"),
            "render_shadows": False, "unmapped_obstacles": "'[]'"}))
    (out/"launch_args.json").write_text(json.dumps(
        {**{k: arg(k) for k in DEFAULTS}, "spawn": tour["spawn"], "initial_pose_map": [round(v, 3) for v in initial],
         "true_start_map": [round(v, 3) for v in start]}, indent=2)+"\n")
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()]
                             + [OpaqueFunction(function=setup)])
