"""/g1_sim: MuJoCo campus + Unitree G1 walking policy as a ROS 2 sensor/actuator node.

Publishes the G1 EDU head sensor suite under the real drivers' default names
(realsense2_camera, livox_ros_driver2), ground-truth odometry/TF, the static map
and /clock. Subscribes /cmd_vel (body-frame vx, vy, yaw rate) for layer 4, the
pretrained 50 Hz walking policy. Simulation time is authoritative (/clock);
when rendering is slower than real time the whole graph slows down with it.
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
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid

from g1_factory.locomotion import G1Locomotion, projected_gravity
from g1_factory.navigation import yaw_from_quat
from g1_factory.planning import OccupancyGrid as Grid
from g1_factory.routes import zone_at
from g1_factory.run import environment_contacts
from g1_factory.scene import build_scene
from g1_factory.sensors import Mid360Lidar, RGBDCamera, imu_reading
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import TOPICS

ROOT = Path(__file__).resolve().parents[2]
OPTICAL = (-0.5, 0.5, -0.5, 0.5)  # x,y,z,w: camera_link (x fwd) -> optical (z fwd, x right, y down)


def add_unmapped_obstacles(scene_path: Path, boxes) -> None:
    """Add collision boxes that are NOT in /map, to exercise LiDAR-based replanning."""
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
        self.camera_hz = float(p("camera_rate_hz", 30.0).value)
        self.lidar_hz = float(p("lidar_rate_hz", 10.0).value)
        self.imu_hz = float(p("imu_rate_hz", 200.0).value)
        self.state_hz = float(p("state_rate_hz", 50.0).value)
        self.clock_hz = float(p("clock_rate_hz", 100.0).value)
        self.realtime_factor = float(p("realtime_factor", 1.0).value)  # <=0: as fast as possible
        width, height = int(p("camera_width", 640).value), int(p("camera_height", 480).value)
        lidar_points = int(p("lidar_points_per_scan", 20000).value)
        self.cmd_timeout = float(p("cmd_timeout_s", 0.5).value)
        self.max_sim_time = float(p("max_sim_time_s", 1800.0).value)
        unmapped = json.loads(p("unmapped_obstacles", "[]").value)
        p("map_resolution", 0.1)
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
        self.joint_qpos, self.joint_dof = list(self.walk.qpos_indices), list(self.walk.dof_indices)

        pub = lambda name, msg: self.create_publisher(msg, name, TOPICS[name][1])
        self.pub_clock = pub("/clock", Clock)
        self.pub_rgb = pub("/camera/camera/color/image_raw", Image)
        self.pub_rgb_info = pub("/camera/camera/color/camera_info", CameraInfo)
        self.pub_depth = pub("/camera/camera/aligned_depth_to_color/image_raw", Image)
        self.pub_depth_info = pub("/camera/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self.pub_cloud = pub("/livox/lidar", PointCloud2)
        self.pub_imu = pub("/livox/imu", Imu)
        self.pub_odom = pub("/odom", Odometry)
        self.pub_joints = pub("/joint_states", JointState)
        self.pub_tf = pub("/tf", TFMessage)
        self.pub_tf_static = pub("/tf_static", TFMessage)
        self.pub_map = pub("/map", OccupancyGrid)
        self.pub_status = pub("/sim/status", String)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, TOPICS["/cmd_vel"][1])

        self.command = np.zeros(3, dtype=np.float32)
        self.last_cmd_time = -math.inf
        self.collision_events, self.collision_names, self.fallen = 0, set(), False
        self.prev_hit = False
        self.travelled, self.last_xy = 0.0, self.data.qpos[:2].copy()
        self.next = {k: 0.0 for k in ("clock", "camera", "lidar", "imu", "state", "status")}
        self.wall_start, self.sim_start = time.monotonic(), self.data.time
        self.timing = {}
        self.publish_static()
        self.get_logger().info(f"G1 sim ready: spawn={spawn} camera {width}x{height}@{self.camera_hz}Hz "
                               f"lidar {lidar_points} rays@{self.lidar_hz}Hz")

    # ------------------------------------------------------------------ inputs
    def on_cmd(self, msg: Twist):
        cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        if not np.isfinite(cmd).all():
            self.get_logger().warn("Ignoring non-finite /cmd_vel")
            return
        # Conservative safety clamp; the path tracker's own limits are lower.
        self.command = np.clip(cmd, [-0.6, -0.3, -1.0], [0.8, 0.3, 1.0]).astype(np.float32)
        self.last_cmd_time = self.data.time

    # --------------------------------------------------------------- publishers
    def publish_static(self):
        t = self.data.time
        cam_pos = self.model.cam_pos[self.camera.camera_id]
        site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "livox_frame")
        lidar_pos = self.model.site_pos[site]
        pitch = math.radians(10)

        def tf(parent, child, xyz, quat):
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = M.stamp(t), parent, child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = map(float, quat)
            return msg
        self.pub_tf_static.publish(TFMessage(transforms=[
            tf("map", "odom", (0, 0, 0), (0, 0, 0, 1)),
            tf("base_link", "camera_link", cam_pos, (0, math.sin(pitch/2), 0, math.cos(pitch/2))),
            tf("camera_link", "camera_color_optical_frame", (0, 0, 0), OPTICAL),
            tf("base_link", "livox_frame", lidar_pos, (0, 0, 0, 1)),
        ]))
        grid = Grid.from_scene(self.meta, float(self.get_parameter('map_resolution').value))
        self.pub_map.publish(M.occupancy_msg(t, "map", grid.origin, grid.resolution, grid.data))

    def timed(self, key, begin):
        total, n = self.timing.get(key, (0.0, 0))
        self.timing[key] = (total+time.perf_counter()-begin, n+1)

    def publish_camera(self, t):
        begin = time.perf_counter()
        rgb, depth = self.camera.render(self.data)
        self.timed("camera_render", begin)
        begin = time.perf_counter()
        frame = "camera_color_optical_frame"
        c = self.camera
        info = M.camera_info_msg(t, frame, c.width, c.height, c.fx, c.fy, c.cx, c.cy)
        self.pub_rgb.publish(M.image_msg(t, frame, rgb, "rgb8"))
        self.pub_rgb_info.publish(info)
        self.pub_depth.publish(M.image_msg(t, frame, np.round(depth*1000).astype(np.uint16), "16UC1"))
        self.pub_depth_info.publish(info)
        self.timed("camera_publish", begin)

    def publish_lidar(self, t):
        begin = time.perf_counter()
        xyz, refl, line, index = self.lidar.scan(self.data)
        self.timed("lidar_raycast", begin)
        begin = time.perf_counter()
        period_ns = 1e9/self.lidar_hz
        point_ns = t*1e9+index*(period_ns/self.lidar.n)
        self.pub_cloud.publish(M.livox_cloud_msg(t, "livox_frame", xyz, refl, line, point_ns))
        self.timed("lidar_publish", begin)

    def publish_imu(self, t):
        gyro, accel = imu_reading(self.model, self.data)
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = M.stamp(t), "livox_frame"
        msg.orientation_covariance[0] = -1.0  # the MID-360 IMU reports no orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, accel)
        self.pub_imu.publish(msg)

    def publish_state(self, t):
        q = self.data.qpos
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = M.stamp(t), "odom", "base_link"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, q[:3])
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x = float(q[3]), float(q[4])
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = float(q[5]), float(q[6])
        rot = self.data.xmat[self.pelvis].reshape(3, 3)
        lin = rot.T@self.data.qvel[:3]  # free-joint linear velocity is world-frame
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, lin)
        ang = self.data.qvel[3:6]  # free-joint angular velocity is body-frame
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, ang)
        self.pub_odom.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, q[:3])
        tf.transform.rotation = msg.pose.pose.orientation
        self.pub_tf.publish(TFMessage(transforms=[tf]))
        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = list(self.walk.joint_names)
        js.position = [float(self.data.qpos[i]) for i in self.joint_qpos]
        js.velocity = [float(self.data.qvel[i]) for i in self.joint_dof]
        js.effort = [float(v) for v in self.data.ctrl[self.walk.actuator_ids]]
        self.pub_joints.publish(js)

    def publish_status(self, t):
        xy = self.data.qpos[:2]
        wall = time.monotonic()-self.wall_start
        self.pub_status.publish(String(data=json.dumps({
            "sim_time_s": round(t, 3), "zone": zone_at(self.meta, xy), "x": float(xy[0]), "y": float(xy[1]),
            "yaw": yaw_from_quat(self.data.qpos[3:7]), "base_z": float(self.data.qpos[2]),
            "collision_events": self.collision_events, "collision_geoms": sorted(self.collision_names),
            "fallen": self.fallen, "travelled_m": round(self.travelled, 3),
            "realtime_factor": round((t-self.sim_start)/max(wall, 1e-6), 3),
            "command": [round(float(v), 3) for v in self.command],
            "timing_ms_mean": {k: round(1000*v[0]/max(v[1], 1), 2) for k, v in self.timing.items()}})))

    # -------------------------------------------------------------------- loop
    def due(self, key, hz, t):
        if t+1e-9 >= self.next[key]:
            self.next[key] = max(self.next[key]+1.0/hz, t)
            return True
        return False

    def step_once(self):
        t = self.data.time
        if self.due("clock", self.clock_hz, t):
            self.pub_clock.publish(Clock(clock=M.stamp(t)))
        if self.walk.counter % self.walk.decimation == 0:
            if t < 1.0 or t-self.last_cmd_time > self.cmd_timeout:
                command = np.zeros(3, dtype=np.float32)  # settle, or stop on stale commands
            else:
                command = self.command
            self.active_command = command
        begin = time.perf_counter()
        self.walk.step(self.active_command)
        self.timed("physics_step", begin)
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
        if self.due("imu", self.imu_hz, t):
            self.publish_imu(t)
        if self.due("state", self.state_hz, t):
            self.publish_state(t)
        if self.due("lidar", self.lidar_hz, t):
            self.publish_lidar(t)
        if self.due("camera", self.camera_hz, t):
            self.publish_camera(t)
        if self.due("status", 2.0, t):
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
