"""/g1_sim on Isaac Sim 5.1: campus USD + Unitree G1 12-DOF + the same ROS 2 interface as MuJoCo.

Run with Isaac Sim's Python (container: /isaac-sim/python.sh), see scripts/run_isaac_sim.sh:

    ./python.sh /ws/g1_factory/isaac/isaac_sim_node.py --campus /ws/outputs/isaac/campus.usd \
        --meta /ws/outputs/isaac/campus_meta.json --spawn -12 3.8 0

* Robot: unitree_rl_gym g1_12dof.urdf (the URDF the walking policy was trained on in
  Isaac Gym), floating base. PD torques are computed here at every physics step
  (dt 5 ms), the policy runs at 50 Hz (every 4th step) - the same structure as the
  training setup - through g1_factory.policy_core, shared with the MuJoCo node.
* Sensors: head RGB-D camera (640x480, HFOV 69.4 deg, pitch -10 deg, D435i-like) and a
  MID-360-like LiDAR emulated with four 100-degree depth cameras sampled with the same
  non-repetitive ray pattern as the MuJoCo ray caster; IMU from base kinematics.
* ROS 2: rclpy bundled with Isaac Sim (Python 3.11, internal Humble libraries),
  publishing through g1_factory.ros2.sim_bridge.SimTopics - identical topic names,
  frames, message layouts and QoS to the MuJoCo /g1_sim.
* Collisions are reported with a footprint proxy against the mapped obstacle
  footprints (/sim/status collision_check = "footprint_proxy").
* Media (g1_factory.media, files under --media-out): --overview renders still views of the
  campus and exits; --video NAME records head-camera (first-person) and chase-camera
  (third-person) frames of every VLN task during the ROS run, for scripts/make_videos.py.
"""
from __future__ import annotations

import argparse
import glob
import json
from collections import deque
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from g1_factory import media  # noqa: E402  (numpy only: safe before SimulationApp)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--campus", type=Path, default=ROOT/"outputs/isaac/campus.usd")
    p.add_argument("--meta", type=Path, default=ROOT/"outputs/isaac/campus_meta.json")
    p.add_argument("--upstream", type=Path, default=ROOT/"external/unitree_rl_gym")
    p.add_argument("--spawn", type=float, nargs=3, default=[-12.0, 3.8, 0.0], metavar=("X", "Y", "YAW"))
    p.add_argument("--unmapped", default="[]", help='JSON list of [x, y, sx, sy, sz] boxes not in /map')
    p.add_argument("--camera-rate", type=float, default=30.0)
    p.add_argument("--lidar-rate", type=float, default=10.0)
    p.add_argument("--lidar-points", type=int, default=20000)
    p.add_argument("--realtime-factor", type=float, default=1.0, help="<= 0: as fast as possible")
    p.add_argument("--max-sim-time", type=float, default=1800.0)
    p.add_argument("--cmd-timeout", type=float, default=0.5)
    p.add_argument("--no-map-tf", action="store_true",
                   help="do not publish map -> odom: the localizer does (VLN with package_frame:=map)")
    p.add_argument("--gui", action="store_true", help="open the Isaac Sim window (default headless)")
    p.add_argument("--smoke-test", type=float, default=0.0,
                   help="seconds: no ROS; stand 2 s then walk forward at 0.3 m/s and report (bring-up check)")
    p.add_argument("--capture", type=int, default=0,
                   help="recognition capture: N poses per topological place (zones and aisles), no ROS")
    p.add_argument("--capture-out", type=Path, default=ROOT/"outputs/recognition/capture")
    p.add_argument("--capture-seed", type=int, default=0)
    p.add_argument("--media-out", type=Path, default=ROOT/"outputs/media", help="folder for --overview and --video")
    p.add_argument("--overview", default="",
                   help="still images, no ROS: comma list of " + ", ".join(media.VIEWS) + " -> overview_<view>.png")
    p.add_argument("--overview-size", type=int, nargs=2, default=[1920, 1080], metavar=("W", "H"))
    p.add_argument("--overview-warmup", type=int, default=120, help="rendered steps before the shot (textures, AA)")
    p.add_argument("--video", default="", metavar="NAME",
                   help="record first- and third-person frames of each VLN task into <media-out>/NAME/")
    p.add_argument("--video-mode", choices=("task", "always"), default="task",
                   help="task: from the agent's 'accepted' to 2 s after done/failed/canceled; always: whole run")
    p.add_argument("--video-size", type=int, nargs=2, default=[1280, 720], metavar=("W", "H"),
                   help="third-person (chase camera) resolution; first-person is the 640x480 head camera")
    p.add_argument("--chase", type=float, nargs=2, default=[2.8, 4.6], metavar=("BACK", "HEIGHT"),
                   help="chase camera: metres behind and above the pelvis")
    args = p.parse_args()
    args.overview = [v for v in args.overview.split(",") if v]
    unknown = [v for v in args.overview if v not in media.VIEWS]
    if unknown:
        p.error(f"--overview: unknown view(s) {unknown}; choose from {list(media.VIEWS)}")
    return args


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402  (must precede every omni/isaacsim import)

app = SimulationApp({"headless": not ARGS.gui, "width": 1280, "height": 720})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


def ensure_python_deps():
    """torch (walking policy) and yaml (its config) ship with Isaac Sim as prebundled extensions;
    put them on sys.path if a plain import fails."""
    def importable():
        try:
            import torch  # noqa: F401
            import yaml  # noqa: F401
            return True
        except ImportError:
            return False
    if importable():
        return
    for ext in ("isaacsim.pip.prebundle", "omni.isaac.ml_archive", "isaacsim.ml_archive", "omni.kit.pip_archive"):
        try:
            enable_extension(ext)
            app.update()
        except Exception:
            pass
        if importable():
            return
    raise ImportError("torch/yaml not importable in Isaac Sim's Python; install with "
                      "/isaac-sim/python.sh -m pip install torch pyyaml")


ensure_python_deps()
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402,F401

from g1_factory.planning import OccupancyGrid  # noqa: E402
from g1_factory.policy_core import JOINT_NAMES, G1PolicyCore, projected_gravity  # noqa: E402
from g1_factory.routes import zone_at  # noqa: E402
from g1_factory.sensors import depth_cameras_to_points, mid360_directions, outside_own_body  # noqa: E402

PHYSICS_DT, DECIMATION = 0.005, 4          # 200 Hz physics, 50 Hz policy (as in training)
IMU_WINDOW = 4                             # IMU acceleration from the velocity change over 20 ms
CAMERA_XYZ, CAMERA_PITCH = (0.12, 0.0, 0.45), math.radians(10.0)
LIDAR_XYZ, CAMERA_HFOV = (0.05, 0.0, 0.42), math.radians(69.4)
LIDAR_CAM_RES, LIDAR_CAM_PITCH, LIDAR_CAM_FOV = 256, math.radians(22.5), math.radians(100.0)


def import_rclpy():
    """Isaac Sim ships rclpy for its Python 3.11; enable the bridge and find it if needed."""
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    try:
        import rclpy  # noqa: F401
    except ImportError:
        distro = os.environ.get("ROS_DISTRO", "humble")
        roots = [os.environ.get("ISAAC_PATH", ""), "/isaac-sim", str(Path(sys.executable).resolve().parents[2])]
        for root in roots:
            for path in glob.glob(f"{root}/exts/isaacsim.ros2.bridge/{distro}/rclpy"):
                sys.path.insert(0, path)
        import rclpy  # noqa: F401
    import rclpy
    return rclpy


def quat_from_matrix(r):
    """3x3 rotation (column vectors) -> unit quaternion w, x, y, z (Shepperd's method)."""
    r = np.asarray(r, dtype=float)
    t = np.trace(r)
    if t > 0:
        k = 2*math.sqrt(1+t)
        q = [0.25*k, (r[2, 1]-r[1, 2])/k, (r[0, 2]-r[2, 0])/k, (r[1, 0]-r[0, 1])/k]
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        k = 2*math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2])
        q = [(r[2, 1]-r[1, 2])/k, 0.25*k, (r[0, 1]+r[1, 0])/k, (r[0, 2]+r[2, 0])/k]
    elif r[1, 1] > r[2, 2]:
        k = 2*math.sqrt(1+r[1, 1]-r[0, 0]-r[2, 2])
        q = [(r[0, 2]-r[2, 0])/k, (r[0, 1]+r[1, 0])/k, 0.25*k, (r[1, 2]+r[2, 1])/k]
    else:
        k = 2*math.sqrt(1+r[2, 2]-r[0, 0]-r[1, 1])
        q = [(r[1, 0]-r[0, 1])/k, (r[0, 2]+r[2, 0])/k, (r[1, 2]+r[2, 1])/k, 0.25*k]
    q = np.array(q)
    return q/np.linalg.norm(q)


def rot_y(a):
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def rot_z(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def quat_matrix_wxyz(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def find_prim(root_path, name):
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(root_path) and prim.GetName() == name:
            return path
    raise RuntimeError(f"prim {name!r} not found under {root_path}")


def make_camera(path, translation, rotation, width, height):
    """Isaac Camera in its 'world' axes convention (x forward, z up), posed relative to its parent link."""
    return Camera(prim_path=path, translation=np.asarray(translation, float), orientation=quat_from_matrix(rotation),
                  resolution=(width, height))


def setup_view_camera(cam, width, height, hfov, far=100.0):
    """After World.reset() (Isaac Sim camera examples): render product and intrinsics."""
    cam.initialize()
    cam.set_focal_length(1.0)
    cam.set_horizontal_aperture(2*math.tan(hfov/2))
    cam.set_vertical_aperture(2*math.tan(hfov/2)*height/width)
    cam.set_clipping_range(0.05, far)


def setup_camera(cam, width, height, hfov):
    """Sensor camera: as setup_view_camera, plus the depth annotator."""
    setup_view_camera(cam, width, height, hfov)
    cam.add_distance_to_image_plane_to_frame()


def aim(cam, eye, target):
    """Move a world-fixed camera (chase view) to eye, looking at target."""
    cam.set_world_pose(position=np.asarray(eye, float), orientation=quat_from_matrix(media.look_at(eye, target)),
                       camera_axes="world")


def depth_of(cam):
    frame = cam.get_current_frame()
    depth = frame.get("distance_to_image_plane") if frame else None
    return None if depth is None else np.asarray(depth, dtype=np.float32)


def urdf_effort_limits(urdf_path):
    import xml.etree.ElementTree as ET
    limits = {}
    for joint in ET.parse(urdf_path).getroot().iter("joint"):
        lim = joint.find("limit")
        if lim is not None and lim.get("effort"):
            limits[joint.get("name")] = float(lim.get("effort"))
    return np.array([limits[n] for n in JOINT_NAMES], dtype=np.float32)


class IsaacG1:
    def __init__(self):
        self.meta = json.loads(ARGS.meta.read_text())
        self.world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
        add_reference_to_stage(str(ARGS.campus.resolve()), "/World/Campus")
        stage = omni.usd.get_context().get_stage()
        self.obstacles = [o["bounds"] for o in self.meta["obstacles"]]
        for k, (x, y, sx, sy, sz) in enumerate(json.loads(ARGS.unmapped)):
            cube = UsdGeom.Cube.Define(stage, f"/World/Unmapped/box_{k}")
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(x, y, sz/2))
            xf.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
            cube.CreateDisplayColorAttr([(0.55, 0.42, 0.26)])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            self.obstacles.append([x-sx/2, y-sy/2, x+sx/2, y+sy/2])
        urdf = ARGS.upstream/"resources/robots/g1_description/g1_12dof.urdf"
        enable_extension("isaacsim.asset.importer.urdf")
        app.update()
        status, config = omni.kit.commands.execute("URDFCreateImportConfig")
        config.merge_fixed_joints = True      # upper body is rigid in the 12-DOF model
        config.fix_base = False
        config.import_inertia_tensor = True
        config.self_collision = False
        config.convex_decomp = False
        config.distance_scale = 1.0
        config.make_default_prim = False
        status, root = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(urdf),
                                                 import_config=config, get_articulation_root=True)
        if not status or not root:
            raise RuntimeError(f"URDF import failed: {urdf}")
        self.root = str(root)
        self.robot = self.world.scene.add(SingleArticulation(prim_path=self.root, name="g1"))
        self.pelvis = find_prim("/"+self.root.strip("/").split("/")[0], "pelvis")  # link prim for the sensors
        self.camera = make_camera(f"{self.pelvis}/head_rgbd", CAMERA_XYZ, rot_y(CAMERA_PITCH), 640, 480)
        self.lidar_rot = [rot_z(k*math.pi/2)@rot_y(LIDAR_CAM_PITCH) for k in range(4)]
        self.lidar_cams = [make_camera(f"{self.pelvis}/livox_cam_{k}", LIDAR_XYZ, r, LIDAR_CAM_RES, LIDAR_CAM_RES)
                           for k, r in enumerate(self.lidar_rot)]
        # Media cameras stand in the world, not on the robot: the chase view (--video) and still views (--overview).
        self.chase = make_camera("/World/media_chase", (0.0, 0.0, 10.0), np.eye(3), *ARGS.video_size) \
            if ARGS.video else None
        width, height = ARGS.overview_size
        self.views = {}
        for name in ARGS.overview:
            region, azimuth, elevation, hfov = media.VIEWS[name]
            eye, target = media.fit_view(media.view_bounds(self.meta, region), azimuth, elevation, hfov, width/height)
            self.views[name] = (make_camera(f"/World/media_view_{name}", eye, media.look_at(eye, target), width,
                                            height), math.radians(hfov))
        self.world.reset()
        setup_camera(self.camera, 640, 480, CAMERA_HFOV)
        for cam in self.lidar_cams:
            setup_camera(cam, LIDAR_CAM_RES, LIDAR_CAM_RES, LIDAR_CAM_FOV)
        if self.chase is not None:
            setup_view_camera(self.chase, *ARGS.video_size, math.radians(70.0), far=300.0)
        for cam, hfov in self.views.values():
            setup_view_camera(cam, width, height, hfov, far=1000.0)
        names = list(self.robot.dof_names)
        missing = [n for n in JOINT_NAMES if n not in names]
        if missing:
            raise RuntimeError(f"G1 joints missing after import: {missing}; got {names}")
        self.idx = np.array([names.index(n) for n in JOINT_NAMES])
        self.core = G1PolicyCore(ARGS.upstream)
        self.effort_limit = urdf_effort_limits(urdf)
        # Explicit PD below: disable the implicit joint drives (zero is unit-independent).
        zeros = np.zeros(len(names), dtype=np.float32)
        self.robot.get_articulation_controller().set_gains(kps=zeros, kds=zeros)
        self.reset(*ARGS.spawn)
        self.fx = 640/(2*math.tan(CAMERA_HFOV/2))
        self.lidar_frame = 0
        self.vel_hist = deque(maxlen=IMU_WINDOW+1)
        self.t = 0.0

    def reset(self, x, y, yaw):
        self.robot.set_world_pose(position=np.array([x, y, 0.80]),
                                  orientation=np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]))
        q = np.zeros(len(self.robot.dof_names), dtype=np.float32)
        q[self.idx] = self.core.default_angles
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(np.zeros_like(q))
        self.core.reset()
        self.targets = self.core.default_angles.copy()
        self.steps = 0

    # --------------------------------------------------------------- physics
    def base_state(self):
        pos, quat = self.robot.get_world_pose()
        rot = quat_matrix_wxyz(quat)
        lin_w, ang_w = self.robot.get_linear_velocity(), self.robot.get_angular_velocity()
        return np.asarray(pos), np.asarray(quat), rot, np.asarray(lin_w), np.asarray(ang_w)

    def step(self, command, render):
        q_all, dq_all = self.robot.get_joint_positions(), self.robot.get_joint_velocities()
        q, dq = q_all[self.idx], dq_all[self.idx]
        if self.steps % DECIMATION == 0:
            pos, quat, rot, lin_w, ang_w = self.base_state()
            self.targets = self.core.act(quat, rot.T@ang_w, q, dq, command, self.t).copy()
        tau = np.clip(self.core.kp*(self.targets-q)-self.core.kd*dq, -self.effort_limit, self.effort_limit)
        self.tau = tau
        self.robot.apply_action(ArticulationAction(joint_efforts=tau, joint_indices=self.idx))
        self.world.step(render=render)
        self.steps += 1
        self.t += PHYSICS_DT

    # --------------------------------------------------------------- sensors
    def rgbd(self):
        rgba = self.camera.get_rgba()
        depth = depth_of(self.camera)
        if rgba is None or depth is None or rgba.size == 0:
            return None
        return np.ascontiguousarray(rgba[..., :3]), depth

    def lidar(self):
        dirs = mid360_directions(ARGS.lidar_points, self.lidar_frame)
        self.lidar_frame += 1
        depths = [depth_of(cam) for cam in self.lidar_cams]
        xyz, idx = depth_cameras_to_points(dirs, depths, self.lidar_rot, LIDAR_CAM_FOV, LIDAR_CAM_RES)
        keep = outside_own_body(xyz, axis_xy=(-LIDAR_XYZ[0], -LIDAR_XYZ[1]))  # the depth cameras see the robot too
        xyz, idx = xyz[keep], idx[keep]
        return xyz, np.full(len(idx), 100.0, np.float32), (idx % 4).astype(np.uint8), idx

    def imu(self, rot, lin_w, ang_w):
        # Velocity difference over the last IMU_WINDOW steps (20 ms): a one-step difference turns physics jitter into
        # metres-per-second-squared noise that throws FAST-LIO2 off; real MEMS IMUs low-pass filter the same way.
        self.vel_hist.append(lin_w.copy())
        span = (len(self.vel_hist)-1)*PHYSICS_DT
        accel_w = ((self.vel_hist[-1]-self.vel_hist[0])/span if span > 0 else np.zeros(3))+np.array([0.0, 0.0, 9.81])
        return rot.T@ang_w, rot.T@accel_w

    def collides(self, xy, margin=0.15):
        x, y = xy
        return any(b[0]-margin <= x <= b[2]+margin and b[1]-margin <= y <= b[3]+margin for b in self.obstacles)


def smoke_test(sim: IsaacG1):
    """Bring-up check without ROS: does G1 stand and walk forward in Isaac Sim?"""
    start = sim.base_state()[0].copy()
    while sim.t < ARGS.smoke_test:
        command = np.zeros(3, np.float32) if sim.t < 2.0 else np.array([0.3, 0.0, 0.0], np.float32)
        render = sim.steps % round(1/(30*PHYSICS_DT)) == 0
        sim.step(command, render)
    pos, quat, *_ = sim.base_state()
    rgbd = sim.rgbd()
    xyz = sim.lidar()[0]
    report = {"sim_time_s": round(sim.t, 2), "start": start.round(3).tolist(), "end": pos.round(3).tolist(),
              "walked_m": round(float(np.linalg.norm(pos[:2]-start[:2])), 3), "base_z": round(float(pos[2]), 3),
              "upright": bool(projected_gravity(quat)[2] < -0.5 and pos[2] > 0.45),
              "rgb": None if rgbd is None else list(rgbd[0].shape),
              "depth_valid": None if rgbd is None else round(float(np.isfinite(rgbd[1]).mean()), 3),
              "lidar_points": int(len(xyz))}
    print("SMOKE", json.dumps(report), flush=True)
    return report


_gui_view = media.ChaseCamera(back=3.6, height=2.4, ahead=1.5, target_dz=0.6)


def follow_camera(pos, rot):
    """--gui: keep the viewport camera behind and above the walking robot (smoothed)."""
    from isaacsim.core.utils.viewports import set_camera_view
    eye, target = _gui_view.update(pos, rot)
    set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")


def video_recorder(node, sim):
    """--video: frame recorder driven by the VLN agent's /vln/feedback events (task start and end).

    Only /vln/feedback is subscribed: a /vln/instruction subscriber would make the external agent believe the VLN
    agent is listening (it sends once it sees a subscriber) and the instruction could be lost."""
    from std_msgs.msg import String
    from g1_factory.ros2.qos import TOPICS
    os.umask(0)  # files stay removable by the host user (container runs as uid 1234)
    recorder = media.FrameRecorder(ARGS.media_out/ARGS.video, ARGS.video_mode, fps=ARGS.camera_rate,
                                   log=node.get_logger().info)

    def on_feedback(msg):
        try:
            event = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(event, dict):
            recorder.on_feedback(event, sim.t)

    node.create_subscription(String, "/vln/feedback", on_feedback, TOPICS["/vln/feedback"][1])
    node.get_logger().info(f"video: {ARGS.video_mode} mode -> {ARGS.media_out/ARGS.video}")
    return recorder


def run_ros(sim: IsaacG1):
    rclpy = import_rclpy()
    from g1_factory.ros2.sim_bridge import Scheduler, SimTopics
    rclpy.init()
    node = rclpy.create_node("g1_sim")
    topics = SimTopics(node, lambda: sim.t)
    schedule = Scheduler(clock=100.0, camera=ARGS.camera_rate, lidar=ARGS.lidar_rate, imu=200.0, state=50.0,
                         status=2.0)
    grid = OccupancyGrid.from_scene(self_meta(sim), 0.1)
    semantic_path = ARGS.meta.parent/"semantic_map.json"
    semantic = semantic_path.read_text() if semantic_path.is_file() else None
    topics.publish_static(0.0, CAMERA_XYZ, CAMERA_PITCH, LIDAR_XYZ, grid, semantic, map_tf=not ARGS.no_map_tf)
    recorder = video_recorder(node, sim) if ARGS.video else None
    chase = media.ChaseCamera(*ARGS.chase)
    node.get_logger().info(f"Isaac Sim G1 ready at {ARGS.spawn}; robot {sim.root}")
    collision_events, prev_hit, fallen, travelled = 0, False, False, 0.0
    last_xy = sim.base_state()[0][:2].copy()
    wall_start = time.monotonic()
    command = np.zeros(3, np.float32)
    try:
        while app.is_running() and rclpy.ok() and sim.t < ARGS.max_sim_time and not fallen:
            begin = time.monotonic()
            for _ in range(DECIMATION):
                t = sim.t
                if sim.steps % DECIMATION == 0:
                    command = topics.active_command(t, ARGS.cmd_timeout)
                camera_due = schedule.due("camera", t)
                lidar_due = schedule.due("lidar", t)
                sim.step(command, render=camera_due or lidar_due)
                t = sim.t
                pos, quat, rot, lin_w, ang_w = sim.base_state()
                if ARGS.gui and camera_due:
                    follow_camera(pos, rot)
                if schedule.due("clock", t):
                    topics.publish_clock(t)
                if schedule.due("imu", t):
                    topics.publish_imu(t, *sim.imu(rot, lin_w, ang_w))
                if schedule.due("state", t):
                    q, dq = sim.robot.get_joint_positions()[sim.idx], sim.robot.get_joint_velocities()[sim.idx]
                    topics.publish_state(t, pos, quat, rot.T@lin_w, rot.T@ang_w, JOINT_NAMES, q, dq, sim.tau)
                if camera_due:
                    frame = sim.rgbd()
                    if frame is not None:
                        topics.publish_camera(t, frame[0], frame[1], sim.fx, sim.fx, 319.5, 239.5)
                    if recorder is not None:
                        if recorder.wants_frame(t) and frame is not None:
                            tpv = media.as_rgb8(sim.chase.get_rgba())
                            if tpv is not None:
                                recorder.add(t, frame[0], tpv)
                        aim(sim.chase, *chase.update(pos, rot))  # pose for the next render
                if lidar_due:
                    xyz, refl, line, index = sim.lidar()
                    period_ns = 1e9/ARGS.lidar_rate
                    topics.publish_lidar(t, xyz, refl, line, t*1e9+index*(period_ns/ARGS.lidar_points))
                hit = sim.collides(pos[:2])
                collision_events += int(hit and not prev_hit)
                prev_hit = hit
                if t > 0.5 and (pos[2] < 0.45 or projected_gravity(quat)[2] > -0.5):
                    fallen = True
                travelled += float(np.linalg.norm(pos[:2]-last_xy))
                last_xy = pos[:2].copy()
                if schedule.due("status", t):
                    wall = time.monotonic()-wall_start
                    yaw = math.atan2(rot[1, 0], rot[0, 0])
                    topics.publish_status(t, zone=zone_at(self_meta(sim), pos[:2]), x=float(pos[0]), y=float(pos[1]),
                                          yaw=yaw, base_z=float(pos[2]), collision_events=collision_events,
                                          collision_geoms=[], collision_check="footprint_proxy", fallen=fallen,
                                          travelled_m=round(travelled, 3), realtime_factor=round(t/max(wall, 1e-6), 3),
                                          simulator="isaac_sim")
            rclpy.spin_once(node, timeout_sec=0)
            if ARGS.realtime_factor > 0:
                sleep = DECIMATION*PHYSICS_DT/ARGS.realtime_factor-(time.monotonic()-begin)
                if sleep > 0:
                    time.sleep(sleep)
        # Final status, so the evaluator records a fall or the end of simulated time before /clock stops.
        pos, quat, rot, *_ = sim.base_state()
        topics.publish_status(sim.t, zone=zone_at(self_meta(sim), pos[:2]), x=float(pos[0]), y=float(pos[1]),
                              yaw=math.atan2(rot[1, 0], rot[0, 0]), base_z=float(pos[2]),
                              collision_events=collision_events, collision_geoms=[],
                              collision_check="footprint_proxy", fallen=fallen, travelled_m=round(travelled, 3),
                              realtime_factor=round(sim.t/max(time.monotonic()-wall_start, 1e-6), 3),
                              simulator="isaac_sim")
        if fallen:
            node.get_logger().error("G1 fell; simulation stopped")
        time.sleep(1.0)  # let DDS deliver the last status
    finally:
        if recorder is not None:
            recorder.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def self_meta(sim):
    """campus_meta.json in the shape g1_factory.planning / routes expect."""
    return sim.meta


def capture(sim: IsaacG1):
    """Recognition capture: head RGB-D + exact camera pose at sampled poses (scored by recognition_eval.py)."""
    try:
        from PIL import Image
    except ImportError:  # Pillow is not guaranteed on Isaac's Python path: fall back to raw arrays
        Image = None
    from g1_factory.recognition import map_from_optical, sample_poses, zone_of
    os.umask(0)  # files stay removable by the host user (container runs as uid 1234)
    out = ARGS.capture_out
    (out/"frames").mkdir(parents=True, exist_ok=True)
    semantic = json.loads((ARGS.meta.parent/"semantic_map.json").read_text())
    (out/"semantic_map.json").write_text(json.dumps(semantic, ensure_ascii=False))
    poses = sample_poses(semantic, sim.meta, ARGS.capture, ARGS.capture_seed)
    index = []
    for k, pose in enumerate(poses):
        sim.reset(pose["x"], pose["y"], pose["yaw"])
        sim.robot.set_linear_velocity(np.zeros(3))
        sim.robot.set_angular_velocity(np.zeros(3))
        for _ in range(12):  # 60 ms of standing (PD + policy, zero command) so render products update
            sim.step(np.zeros(3, np.float32), render=True)
        frame = sim.rgbd()
        if frame is None:
            continue
        pos, quat, *_ = sim.base_state()
        name = f"{k:04d}_{pose['place']}"
        if Image is not None:
            Image.fromarray(frame[0]).save(out/"frames"/f"{name}.jpg", quality=92)
        else:
            np.save(out/"frames"/f"{name}_rgb.npy", frame[0])
        np.save(out/"frames"/f"{name}_depth.npy", frame[1].astype(np.float16))
        index.append({**pose, "frame": name, "base_pos": [float(v) for v in pos], "base_quat": [float(v) for v in quat],
                      "zone": zone_of(semantic, pos[:2]),
                      "map_from_optical": map_from_optical(pos, quat).round(5).tolist()})
        print(f"CAPTURE {k+1}/{len(poses)} {name}", flush=True)
    (out/"index.json").write_text(json.dumps({"simulator": "isaac_sim_5.1", "per_place": ARGS.capture,
                                              "seed": ARGS.capture_seed, "captures": index}, indent=1))
    print(f"CAPTURE DONE {len(index)} frames -> {out}", flush=True)


def overview(sim: IsaacG1):
    """--overview: still images of the campus from cameras framed by g1_factory.media.fit_view, no ROS.

    The robot stands at --spawn. Rendering a number of steps first lets textures stream in and the RTX
    anti-aliasing settle; a view that still returns nothing is reported and skipped."""
    os.umask(0)
    ARGS.media_out.mkdir(parents=True, exist_ok=True)
    zero = np.zeros(3, np.float32)
    for _ in range(ARGS.overview_warmup):
        sim.step(zero, render=True)
    done = 0
    for name, (cam, _) in sim.views.items():
        rgb = None
        for _ in range(30):
            rgb = media.as_rgb8(cam.get_rgba())
            if rgb is not None and rgb.max() > 0:
                break
            sim.step(zero, render=True)
        if rgb is None:
            print(f"OVERVIEW {name} FAIL: the camera returned no image", flush=True)
            continue
        path = media.save_image(ARGS.media_out/f"overview_{name}.png", rgb)
        done += 1
        print(f"OVERVIEW {name} -> {path} ({rgb.shape[1]}x{rgb.shape[0]})", flush=True)
    print(f"OVERVIEW DONE {done}/{len(sim.views)}", flush=True)


def main():
    sim = IsaacG1()
    try:
        if ARGS.overview:
            overview(sim)
        elif ARGS.capture > 0:
            capture(sim)
        elif ARGS.smoke_test > 0:
            report = smoke_test(sim)
            ok = report["upright"] and report["walked_m"] > 0.5 and report["lidar_points"] > 1000
            print("SMOKE", "PASS" if ok else "FAIL", flush=True)
        else:
            run_ros(sim)
    finally:
        app.close()


if __name__ == "__main__":
    main()
