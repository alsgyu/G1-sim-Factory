"""/g1_sim: MuJoCo campus + Unitree G1 walking policy as a ROS 2 sensor/actuator node.

Publishes the G1 EDU head sensor suite under the real drivers' default names
(realsense2_camera, livox_ros_driver2), ground-truth odometry/TF, the SLAM-style
occupancy map and /clock. Subscribes /cmd_vel (body-frame vx, vy, yaw rate) for
layer 4, the pretrained 50 Hz walking policy. Simulation time is authoritative
(/clock); when rendering is slower than real time the whole graph slows down
with it. The Isaac Sim node (g1_factory.isaac.isaac_sim_node) publishes the same
topics through the same SimTopics helper.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node

from g1_factory.locomotion import G1Locomotion, projected_gravity
from g1_factory.navigation import yaw_from_quat
from g1_factory.planning import OccupancyGrid as Grid
from g1_factory.routes import zone_at
from g1_factory.run import environment_contacts
from g1_factory.scene import build_scene
from g1_factory.sensors import DriftingOdometry, Mid360Lidar, RGBDCamera, imu_reading
from g1_factory.ros2.sim_bridge import Scheduler, SimTopics

ROOT = Path(__file__).resolve().parents[2]


def add_unmapped_obstacles(scene_path: Path, boxes) -> None:
    """Add collision boxes that are NOT in /map, to exercise LiDAR-based avoidance."""
    tree = ET.parse(scene_path)
    world = tree.getroot().find("worldbody")
    for k, (x, y, sx, sy, sz) in enumerate(boxes):
        ET.SubElement(world, "geom", name=f"factory_unmapped_{k}", type="box",
                      pos=f"{x} {y} {sz/2}", size=f"{sx/2} {sy/2} {sz/2}", rgba=".55 .42 .26 1",
                      contype="1", conaffinity="1")
    tree.write(scene_path, encoding="utf-8", xml_declaration=True)


class G1SimNode(Node):
    def __init__(self):
        super().__init__("g1_sim")
        p = self.declare_parameter
        self.output = Path(p("output_dir", str(ROOT/"outputs/ros2_sim")).value)
        spawn = list(p("spawn", [-12.0, 3.8, 0.0]).value)
        self.realtime_factor = float(p("realtime_factor", 1.0).value)  # <=0: as fast as possible
        width, height = int(p("camera_width", 640).value), int(p("camera_height", 480).value)
        lidar_points = int(p("lidar_points_per_scan", 20000).value)
        self.cmd_timeout = float(p("cmd_timeout_s", 0.5).value)
        self.max_sim_time = float(p("max_sim_time_s", 1800.0).value)
        unmapped = json.loads(p("unmapped_obstacles", "[]").value)
        map_resolution = float(p("map_resolution", 0.1).value)
        map_tf = bool(p("map_tf", True).value)  # false: a localizer owns map -> odom (package_frame:=map)
        drift = [float(v) for v in str(p("odom_drift", "").value).replace(",", " ").split()]
        # "scale yaw_deg_per_m": /odom drifts like leg odometry, the truth goes to /sim/ground_truth
        self.drift = DriftingOdometry(*drift) if drift else None
        self.schedule = Scheduler(clock=float(p("clock_rate_hz", 100.0).value),
                                  camera=float(p("camera_rate_hz", 30.0).value),
                                  lidar=float(p("lidar_rate_hz", 10.0).value),
                                  imu=float(p("imu_rate_hz", 200.0).value),
                                  state=float(p("state_rate_hz", 50.0).value), status=2.0)
        self.output.mkdir(parents=True, exist_ok=True)

        self.meta = build_scene(ROOT/"configs/factory.yaml", ROOT/"external/unitree_rl_gym", self.output/"scene.xml")
        if unmapped:
            add_unmapped_obstacles(Path(self.meta["scene_path"]), unmapped)
        self.model = mujoco.MjModel.from_xml_path(self.meta["scene_path"])
        self.data = mujoco.MjData(self.model)
        self.walk = G1Locomotion(self.model, self.data, ROOT/"external/unitree_rl_gym")
        self.walk.reset(spawn[:2], spawn[2])
        self.camera = RGBDCamera(self.model, "head_rgbd", width, height,
                                 shadows=bool(p("render_shadows", False).value))
        self.lidar = Mid360Lidar(self.model, points_per_scan=lidar_points)
        self.pelvis = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.topics = SimTopics(self, lambda: self.data.time)
        self.collision_events, self.collision_names, self.fallen = 0, set(), False
        self.prev_hit = False
        self.travelled, self.last_xy = 0.0, self.data.qpos[:2].copy()
        self.wall_start, self.sim_start = time.monotonic(), self.data.time
        site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "livox_frame")
        self.topics.publish_static(self.data.time, self.model.cam_pos[self.camera.camera_id], math.radians(10),
                                   self.model.site_pos[site], Grid.from_scene(self.meta, map_resolution),
                                   map_tf=map_tf)
        self.get_logger().info(f"G1 sim ready: spawn={spawn} camera {width}x{height}@{self.schedule.rates['camera']}Hz "
                               f"lidar {lidar_points} rays@{self.schedule.rates['lidar']}Hz")

    # --------------------------------------------------------------- publishers
    def publish_camera(self, t):
        begin = time.perf_counter()
        rgb, depth = self.camera.render(self.data)
        self.topics.timed("camera_render", begin)
        c = self.camera
        self.topics.publish_camera(t, rgb, depth, c.fx, c.fy, c.cx, c.cy)

    def publish_lidar(self, t):
        begin = time.perf_counter()
        xyz, refl, line, index = self.lidar.scan(self.data)
        self.topics.timed("lidar_raycast", begin)
        period_ns = 1e9/self.schedule.rates["lidar"]
        self.topics.publish_lidar(t, xyz, refl, line, t*1e9+index*(period_ns/self.lidar.n))

    def publish_state(self, t):
        q, v = self.data.qpos, self.data.qvel
        rot = self.data.xmat[self.pelvis].reshape(3, 3)
        position, quat = q[:3], q[3:7]
        if self.drift is not None:
            self.topics.publish_truth(t, position, quat)
            yaw = yaw_from_quat(quat)
            x, y, odom_yaw = self.drift.update(float(q[0]), float(q[1]), yaw)
            h = (odom_yaw-yaw)/2  # turn the true orientation about z by the heading drift
            w, z = math.cos(h), math.sin(h)
            quat = np.array([w*quat[0]-z*quat[3], w*quat[1]-z*quat[2], w*quat[2]+z*quat[1], w*quat[3]+z*quat[0]])
            position = np.array([x, y, q[2]])
        # Free-joint linear velocity is world-frame; angular velocity is body-frame.
        self.topics.publish_state(t, position, quat, rot.T@v[:3], v[3:6], self.walk.joint_names,
                                  q[self.walk.qpos_indices], v[self.walk.dof_indices],
                                  self.data.ctrl[self.walk.actuator_ids])

    def publish_status(self, t):
        xy = self.data.qpos[:2]
        wall = time.monotonic()-self.wall_start
        self.topics.publish_status(
            t, zone=zone_at(self.meta, xy), x=float(xy[0]), y=float(xy[1]), yaw=yaw_from_quat(self.data.qpos[3:7]),
            base_z=float(self.data.qpos[2]), collision_events=self.collision_events,
            collision_geoms=sorted(self.collision_names), collision_check="mujoco_contacts", fallen=self.fallen,
            travelled_m=round(self.travelled, 3), realtime_factor=round((t-self.sim_start)/max(wall, 1e-6), 3),
            simulator="mujoco")

    # -------------------------------------------------------------------- loop
    def step_once(self):
        t = self.data.time
        due = self.schedule.due
        if due("clock", t):
            self.topics.publish_clock(t)
        if self.walk.counter % self.walk.decimation == 0:
            self.active_command = self.topics.active_command(t, self.cmd_timeout)
        begin = time.perf_counter()
        self.walk.step(self.active_command)
        self.topics.timed("physics_step", begin)
        t = self.data.time
        contacts = environment_contacts(self.model, self.data)
        hit = bool(contacts)
        self.collision_events += int(hit and not self.prev_hit)
        self.collision_names.update(contacts)
        self.prev_hit = hit
        if t > 0.5 and (self.data.qpos[2] < 0.45 or projected_gravity(self.data.qpos[3:7])[2] > -0.5):
            self.fallen = True
        xy = self.data.qpos[:2].copy()
        self.travelled += float(np.linalg.norm(xy-self.last_xy))
        self.last_xy = xy
        if due("imu", t):
            self.topics.publish_imu(t, *imu_reading(self.model, self.data))
        if due("state", t):
            self.publish_state(t)
        if due("lidar", t):
            self.publish_lidar(t)
        if due("camera", t):
            self.publish_camera(t)
        if due("status", t):
            self.publish_status(t)

    def run(self):
        self.active_command = np.zeros(3, dtype=np.float32)
        while rclpy.ok() and self.data.time < self.max_sim_time and not self.fallen:
            begin = time.monotonic()
            for _ in range(10):  # 20 ms of simulation between executor polls
                self.step_once()
            rclpy.spin_once(self, timeout_sec=0)
            if self.realtime_factor > 0:
                sleep = 0.02/self.realtime_factor-(time.monotonic()-begin)
                if sleep > 0:
                    time.sleep(sleep)
        self.publish_status(self.data.time)
        if self.fallen:
            self.get_logger().error("G1 fell; simulation stopped")
        time.sleep(1.0)  # let DDS deliver the last status before the node goes away


def main():
    rclpy.init()
    node = G1SimNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.camera.close()
        node.lidar.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
