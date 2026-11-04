"""naive-pp on the G1: instruction -> place on the robot's map -> topological route -> A* -> path follower.

    # Isaac Sim (terminal 1, G1-sim-Factory's script), the simulator does not publish map -> odom:
    bash G1-sim-Factory/scripts/run_isaac_sim.sh --gui --spawn -12 3.8 0 --no-map-tf
    # terminal 2 (this repository's root): localizer + naive_pp + path_follower on the map package, then send
    # instructions. In the container this repository is /pp and G1-sim-Factory /ws (scripts/g1_docker.sh).
    bash scripts/g1_docker.sh ros2 launch launch/naive_pp.launch.py initial_pose:=0,0,0
    bash scripts/g1_docker.sh python3 /ws/scripts/send_instruction.py "Go to the operations control room ..."

    # MuJoCo instead of Isaac Sim (development), one instruction, recorded, then exit:
    G1_DDS=cyclonedds bash scripts/g1_docker.sh ros2 launch launch/naive_pp.launch.py sim:=true \
        instruction:="Go to the pump room" output_dir:=/pp/outputs/run1

Nodes: map_file (the package's SLAM map on /slam_map, rooms and doorways on /slam/semantic_map), localizer
(TF map -> odom from LiDAR scans), naive_pp (planner), path_follower (/plan -> /cmd_vel), minimap (web page at
http://localhost:8765, minimap:=false to leave out), optionally g1_sim (MuJoCo, sim:=true) and naive_pp_client
(instruction:=..., the launch ends when it does). map_file, localizer, path_follower and g1_sim are G1-sim-Factory's.
"""
import json
import math
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from naive_pp import SIM  # noqa: E402  (puts g1_factory on the path)
from g1_factory.localization import compose, inverse  # noqa: E402
from g1_factory.map_package import world_from_map  # noqa: E402

DEFAULTS = {
    "package": (str(HERE/"maps/campus_dev"), "Map package: the robot's SLAM map + a person's annotations"),
    "initial_pose": ("0,0,0", "Rough robot pose x,y,yaw (m, rad) in the package's map frame, within 1.5 m and 20 deg. "
                              "0,0,0 = where the robot stood when it started mapping (the map origin). Empty: wait for "
                              "/initialpose (RViz 2D Pose Estimate). spawn (simulation only): the spawn moved into the "
                              "map frame with the package's evaluation.json, standing in for a person pointing at it"),
    "sim": ("false", "false: Isaac Sim (run_isaac_sim.sh ... --no-map-tf) or the robot provides the topics; "
                     "true: start the MuJoCo g1_sim (development)"),
    "spawn": ("-12.0,3.8,0.0", "sim:=true only: MuJoCo spawn x,y,yaw in the simulator frame"),
    "camera_rate_hz": ("2.0", "sim:=true only: head camera rate (naive-pp does not use images)"),
    "realtime_factor": ("1.0", "sim:=true only: <= 0 runs MuJoCo as fast as possible"),
    "unmapped_obstacles": ("[]", "sim:=true only: JSON [[x, y, size_x, size_y, size_z], ...] boxes (simulator frame) "
                                 "that exist in physics and LiDAR but not on the map, to test replanning"),
    "use_sim_time": ("true", "Follow /clock (MuJoCo, Isaac Sim); false on a real robot"),
    "remaps": ("", "Comma-separated topic remaps for every node, e.g. /livox/lidar:=/point_cloud"),
    "max_speed": ("0.4", "path_follower forward speed limit (m/s)"),
    "goal_mode": ("center", "center: stand in the middle of the room | entrance: just inside its main doorway"),
    "minimap": ("true", "Serve the live minimap (map, room names, robot, plan, target room) at http://localhost:PORT"),
    "minimap_port": ("8765", "Port of the minimap web page"),
    "instruction": ("", "Send this instruction once localized, record the run in output_dir, then stop everything"),
    "output_dir": (str(HERE/"outputs/run"), "Where naive_pp_client writes feedback, trajectory and summary"),
    "timeout_s": ("1200.0", "naive_pp_client gives up after this many simulation seconds"),
}


def node(module, name, params, remaps=()):
    args = ["python3", "-m", module, "--ros-args", "-r", f"__node:={name}"]
    for remap in remaps:
        args += ["-r", remap]
    for key, value in params.items():
        if isinstance(value, (list, tuple)):
            value = "[" + ",".join(f"{float(v):.6f}" for v in value) + "]"
        elif isinstance(value, bool):
            value = str(value).lower()
        args += ["-p", f"{key}:={value}"]
    env = {"PYTHONPATH": os.pathsep.join([str(HERE), str(SIM), os.environ.get("PYTHONPATH", "")]),
           "G1_SIM_ROOT": str(SIM)}
    return ExecuteProcess(cmd=args, name=name, output="screen", cwd=str(SIM), additional_env=env)


def setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    package = str(Path(arg("package")).resolve())
    sim_time = {"use_sim_time": arg("use_sim_time").lower() == "true"}
    remaps = [r.strip() for r in arg("remaps").split(",") if r.strip()]
    if any(":=" not in r for r in remaps):
        raise ValueError("remaps must look like /from:=/to[,/from2:=/to2]")
    spawn = [float(v) for v in arg("spawn").replace(",", " ").split()]
    initial = arg("initial_pose")
    if initial == "spawn":
        w = world_from_map(package)
        initial = ",".join(f"{v:.3f}" for v in compose(inverse((w[0, 2], w[1, 2], math.atan2(w[1, 0], w[0, 0]))),
                                                       tuple(spawn)))
    actions = [
        node("g1_factory.ros2.map_file_node", "map_file", {**sim_time, "package": package, "frame": "map",
                                                            "topic": "/slam_map"}),
        node("g1_factory.ros2.localizer_node", "localizer", {**sim_time, "package": package,
                                                             "initial_pose": f"'{initial}'"}, remaps),
        node("naive_pp.ros.pp_node", "naive_pp", {**sim_time, "package": package, "goal_mode": arg("goal_mode")},
             remaps),
        node("g1_factory.ros2.follower_node", "path_follower", {**sim_time, "max_speed_mps": arg("max_speed"),
                                                                "goal_timeout_s": 3600.0}, remaps),
    ]
    if arg("minimap").lower() == "true":
        actions.append(node("naive_pp.ros.minimap_node", "minimap", {**sim_time, "package": package,
                                                                     "port": int(arg("minimap_port"))}, remaps))
    if arg("sim").lower() == "true":
        out = Path(arg("output_dir"))
        actions.insert(0, node("g1_factory.ros2.sim_node", "g1_sim", {
            "use_sim_time": False, "output_dir": str(out/"sim"), "spawn": spawn, "map_tf": False,
            "camera_rate_hz": arg("camera_rate_hz"), "realtime_factor": arg("realtime_factor"),
            "render_shadows": False,
            # Quoted so ROS parses it as a YAML string, not a nested list.
            "unmapped_obstacles": "'" + arg("unmapped_obstacles") + "'"}))
    if arg("instruction"):
        out = Path(arg("output_dir"))
        out.mkdir(parents=True, exist_ok=True)
        (out/"instruction.txt").write_text(arg("instruction")+"\n", encoding="utf-8")
        (out/"launch_args.json").write_text(json.dumps({**{k: arg(k) for k in DEFAULTS}, "package": package,
                                                        "initial_pose_used": initial}, indent=1)+"\n")
        client = node("naive_pp.ros.client_node", "naive_pp_client",
                      {**sim_time, "instruction_file": str(out/"instruction.txt"), "output_dir": str(out),
                       "timeout_s": arg("timeout_s")}, remaps)
        actions += [client, RegisterEventHandler(OnProcessExit(
            target_action=client, on_exit=[EmitEvent(event=Shutdown(reason="instruction finished"))]))]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()]
                             + [OpaqueFunction(function=setup)])
