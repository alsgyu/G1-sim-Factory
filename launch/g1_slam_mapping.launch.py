"""SLAM mapping run: the G1 walks the campus once and builds its own map with FAST-LIO2.

    # terminal 1 (Isaac Sim, at the tour's start pose)
    bash scripts/run_isaac_sim.sh --spawn -12 3.8 0
    # terminal 2
    bash scripts/g1_docker.sh ros2 launch launch/g1_slam_mapping.launch.py name:=campus

Nodes: fast_lio (FAST-LIO2, configs/fast_lio_g1_sim.yaml), slam_map (occupancy grid + 3-D cloud, saves the map
package), mapping_tour (walks configs/slam_mapping.yaml), global_planner and path_follower (+ g1_sim with
sim:=true, the MuJoCo development graph). The launch ends when the map is saved to maps/<name>/.
The tour plans on the simulator's /map only to steer the robot (a person with a joystick on a real robot); the
SLAM map itself comes from the LiDAR and IMU alone.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]

DEFAULTS = {
    "name": ("campus", "Map name: the package is written to <output_root>/<name>/"),
    "output_root": (str(ROOT/"maps"), "Folder for map packages"),
    "tour": ("campus", "Tour in tour_file"),
    "tour_file": (str(ROOT/"configs/slam_mapping.yaml"), "Tours (spawn + waypoints)"),
    "fast_lio_config": (str(ROOT/"configs/fast_lio_g1_sim.yaml"), "FAST-LIO2 parameters"),
    "evaluate": ("true", "Simulation: compare with the ground-truth map and pose, write map_world.yaml"),
    "sim": ("false", "true: start the MuJoCo g1_sim (development only); default: Isaac Sim provides the topics"),
    "camera_rate_hz": ("2.0", "MuJoCo only: the head camera is not needed for mapping"),
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
    sim_time = {"use_sim_time": True}
    tour = yaml.safe_load(Path(arg("tour_file")).read_text())["tours"][arg("tour")]
    fast_lio = ExecuteProcess(cmd=["ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args", "-r", "__node:=fast_lio",
                                   "--params-file", arg("fast_lio_config"), "-p", "use_sim_time:=true"],
                              name="fast_lio", output="screen", cwd="/tmp")
    tour_node = node("mapping_tour_node", "mapping_tour", {**sim_time, "tour_file": arg("tour_file"), "tour": arg("tour")})
    actions = [
        fast_lio,
        node("slam_map_node", "slam_map", {**sim_time, "name": arg("name"), "output_root": arg("output_root"),
                                           "evaluate": arg("evaluate").lower() == "true"}),
        node("planner_node", "global_planner", sim_time),
        node("follower_node", "path_follower", sim_time),
        tour_node,
        RegisterEventHandler(OnProcessExit(target_action=tour_node,
                                           on_exit=[EmitEvent(event=Shutdown(reason="mapping finished"))])),
    ]
    if arg("sim").lower() == "true":
        actions.insert(0, node("sim_node", "g1_sim", {
            "use_sim_time": False, "output_dir": str(ROOT/"outputs"/f"slam_{arg('name')}"/"sim"),
            "spawn": tour["spawn"], "camera_rate_hz": arg("camera_rate_hz"), "realtime_factor": arg("realtime_factor"),
            "render_shadows": False, "unmapped_obstacles": "'[]'"}))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()]
                             + [OpaqueFunction(function=setup)])
