"""slam_map: the robot's own map from LiDAR-inertial SLAM (FAST-LIO2), saved as a map package to share.

Subscribes
  /cloud_registered, /Odometry   FAST-LIO2 output (frame camera_init = LiDAR/IMU frame at start-up)
  /livox/imu                     gravity at start-up for levelling; relayed RELIABLE on /slam/imu because
                                 FAST-LIO2 subscribes the IMU reliably (the real Livox driver publishes reliably)
  /mapping/done                  save now (the mapping tour publishes it at the end); Ctrl-C also saves
  /odom, /map, /tf_static        simulation only (evaluate:=true): ground truth for the trajectory error, the map
                                 comparison and a copy of the map in the simulator frame
Publishes /slam/map (OccupancyGrid, frame slam_map, latched, every 2 s) and /slam/status.

Saves <output_root>/<name>/ (g1_factory.slam_map): map.yaml + map.pgm (ROS map_server / Nav2 format),
map.png, cloud.pcd, trajectory.csv, meta.json; with ground truth also map_world.yaml/.pgm (same map in the
simulator frame, for the VLN in simulation), semantic_truth.json (the simulator's semantic map in the robot's map
frame, only to check annotations against), evaluation.json and comparison.png. The package's semantic_map.json is
built from a person's annotations (scripts/build_map_package.py).
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from g1_factory import slam_map as S
from g1_factory.ros2 import messages as M
from g1_factory.ros2.qos import IMU, LATCHED, TOPICS

ROOT = Path(__file__).resolve().parents[2]
RELIABLE = QoSProfile(depth=200)


def path_viewpoints(xy: np.ndarray, spacing=1.0) -> np.ndarray:
    """Positions along the walked path, one every `spacing` metres."""
    travelled = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    marks = np.floor(travelled/spacing)
    return xy[np.r_[True, marks[1:] != marks[:-1]]]


class SlamMapNode(Node):
    def __init__(self):
        super().__init__("slam_map")
        p = self.declare_parameter
        self.name = p("name", "campus").value
        self.out_root = Path(p("output_root", str(ROOT/"maps")).value)
        self.evaluate = bool(p("evaluate", True).value)
        self.semantic_path = Path(p("semantic", str(ROOT/"outputs/isaac/semantic_map.json")).value)
        self.gravity_samples = int(p("gravity_samples", 200).value)
        self.grid = S.SlamGrid(resolution=float(p("resolution", 0.1).value),
                               band=(float(p("obstacle_min_z_m", 0.15).value), float(p("obstacle_max_z_m", 1.6).value)),
                               min_hits=int(p("min_hits", 2).value))  # obstacle voxels per column
        self.accel, self.R = [], None
        self.floor_votes, self.floor = [], None
        self.traj, self.truth, self.truth_map, self.lidar_xyz = [], [], None, None
        self.traj_last_raw = None  # latest FAST-LIO2 position, camera_init frame
        self.scans, self.saved, self.started = 0, False, time.time()
        self.clouds = 0  # /cloud_registered messages received (FAST-LIO2 is running)

        self.imu_relay = self.create_publisher(Imu, "/slam/imu", RELIABLE)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, IMU)
        self.create_subscription(PointCloud2, "/cloud_registered", self.on_cloud, QoSProfile(depth=20))
        self.create_subscription(Odometry, "/Odometry", self.on_slam_odom, QoSProfile(depth=50))
        self.create_subscription(String, "/mapping/done", lambda _: self.save(), TOPICS["/vln/instruction"][1])
        if self.evaluate:
            self.create_subscription(Odometry, "/odom", self.on_truth, TOPICS["/odom"][1])
            self.create_subscription(OccupancyGrid, "/map", self.on_truth_map, LATCHED)
            self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, TOPICS["/tf_static"][1])
        self.pub_map = self.create_publisher(OccupancyGrid, "/slam/map", LATCHED)
        self.pub_status = self.create_publisher(String, "/slam/status", TOPICS["/nav/status"][1])
        self.create_timer(10.0, self.publish_map)  # preview without the visibility test (saved map has it)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    # ------------------------------------------------------------------ inputs
    def on_imu(self, msg: Imu):
        self.imu_relay.publish(msg)
        if self.R is None:
            a = msg.linear_acceleration
            self.accel.append((a.x, a.y, a.z))
            if len(self.accel) >= self.gravity_samples:
                self.R = S.gravity_alignment(np.array(self.accel))
                tilt = math.degrees(math.acos(np.clip(self.R[2, 2], -1, 1)))
                self.get_logger().info(f"levelled with gravity from {len(self.accel)} IMU samples (tilt {tilt:.2f} deg)")

    def levelled(self, xyz):
        return np.asarray(xyz)@self.R.T

    def on_cloud(self, msg: PointCloud2):
        if self.R is None or self.saved:
            return
        self.clouds += 1
        pts = self.levelled(M.cloud_xyz(msg))
        if self.floor is None:
            sensor = self.levelled(self.traj_last_raw) if self.traj_last_raw is not None else np.zeros(3)
            h = S.floor_height(pts, sensor)
            if h is not None:
                self.floor_votes.append(h)
            if len(self.floor_votes) >= 5:
                self.floor = float(np.median(self.floor_votes))
                self.get_logger().info(f"floor {-self.floor:.2f} m below the LiDAR at start-up")
            return
        pts[:, 2] -= self.floor
        self.grid.add(pts)
        self.scans += 1
        if self.scans == 1:
            self.status("tracking", floor_below_lidar_m=round(-self.floor, 2))

    def on_slam_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.traj_last_raw = np.array([p.x, p.y, p.z])
        if self.R is None or self.floor is None or self.saved:
            return
        xyz = self.levelled(self.traj_last_raw)-np.array([0, 0, self.floor])
        heading = self.R@M.quaternion_matrix(q)[:, 0]
        self.traj.append((M.to_seconds(msg.header.stamp), *xyz, math.atan2(heading[1], heading[0])))

    def on_truth(self, msg: Odometry):
        p = msg.pose.pose.position
        self.truth.append((M.to_seconds(msg.header.stamp), p.x, p.y, M.quaternion_yaw(msg.pose.pose.orientation)))

    def on_truth_map(self, msg: OccupancyGrid):
        self.truth_map = (M.occupancy_to_numpy(msg), (msg.info.origin.position.x, msg.info.origin.position.y),
                          msg.info.resolution)

    def on_tf_static(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id == "base_link" and t.child_frame_id == "livox_frame":
                self.lidar_xyz = np.array([t.transform.translation.x, t.transform.translation.y])

    def publish_map(self):
        if self.saved:
            return
        if self.scans == 0:  # say what is missing instead of mapping nothing in silence
            if self.R is None:
                reason = f"no IMU on /livox/imu yet ({len(self.accel)} samples): the simulator or LiDAR driver is not publishing it"
            elif self.clouds == 0:
                reason = ("no /cloud_registered from FAST-LIO2 yet: check the fast_lio process in this log "
                          "(missing package = rebuild the image, docker/Dockerfile.ros2)")
            else:
                reason = f"FAST-LIO2 publishes ({self.clouds} scans) but the floor was not found under the LiDAR yet"
            self.get_logger().warning(reason)
            self.status("waiting", message=reason)
            return
        grid, origin = self.grid.occupancy()
        self.pub_map.publish(M.occupancy_msg(self.now(), "slam_map", origin, self.grid.resolution, grid))
        self.status("mapping", scans=self.scans, poses=len(self.traj),
                    known_m2=round(float((grid != S.UNKNOWN).sum())*self.grid.resolution**2))

    def status(self, state, **extra):
        if not rclpy.ok():  # shutting down (Ctrl-C or the launch ending): nobody is listening any more
            return
        self.pub_status.publish(String(data=json.dumps({"state": state, "sim_time_s": round(self.now(), 2), **extra})))

    # ------------------------------------------------------------------ saving
    def world_from_map(self):
        """Simulator frame <- robot map frame, from the ground-truth pose when SLAM started (simulation only)."""
        if not self.traj or not self.truth or self.lidar_xyz is None:
            return None
        truth = np.array(self.truth)
        k = int(np.argmin(np.abs(truth[:, 0]-self.traj[0][0])))
        _, x, y, yaw = truth[k]
        lx = x+math.cos(yaw)*self.lidar_xyz[0]-math.sin(yaw)*self.lidar_xyz[1]
        ly = y+math.sin(yaw)*self.lidar_xyz[0]+math.cos(yaw)*self.lidar_xyz[1]
        _, sx, sy, _, syaw = self.traj[0]  # same instant in the robot's map frame
        return S.se2(lx, ly, yaw)@np.linalg.inv(S.se2(sx, sy, syaw))

    def save(self):
        if self.saved:
            return
        if self.scans == 0:
            self.get_logger().error("no SLAM scans received; nothing to save")
            self.status("failed", message="no scans")
            return
        self.saved = True
        folder = self.out_root/self.name
        folder.mkdir(parents=True, exist_ok=True)
        res = self.grid.resolution
        traj = np.array(self.traj)
        built = self.grid.build(viewpoints=path_viewpoints(traj[:, 1:3]) if len(traj) else None)
        grid, origin, cloud, R, dz = built["grid"], built["origin"], built["cloud"], built["R"], built["dz"]
        tilt = math.degrees(math.acos(np.clip(R[2, 2], -1, 1)))
        self.get_logger().info(f"floor plane levelling: {tilt:.2f} deg, {dz:+.3f} m")
        if len(traj):
            traj[:, 1:4] = traj[:, 1:4]@R.T-np.array([0.0, 0.0, dz])
            self.traj = [tuple(row) for row in traj]
        S.write_map_yaml(folder, grid, origin, res)
        cells = np.floor((traj[:, [2, 1]]-np.array(origin)[::-1])/res).astype(int) if len(traj) else None
        S.preview(grid, cells).save(folder/"map.png")
        S.write_pcd(folder/"cloud.pcd", cloud)
        with (folder/"trajectory.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_m", "y_m", "z_m", "yaw_rad"])
            w.writerows([[round(v, 4) for v in row] for row in self.traj])
        meta = {"name": self.name, "frame": "map: origin on the floor under the LiDAR at start-up, x = start heading, "
                                           "z up, floor at z = 0 (levelled on the floor plane), units m",
                "floor_levelling_deg": round(tilt, 3),
                "slam": "FAST-LIO2 (LiDAR-inertial odometry, ROS 2 branch) + g1_factory.slam_map occupancy",
                "sensor": "Livox MID-360 (head), internal IMU", "resolution_m": res, "origin": list(origin),
                "size_cells": list(grid.shape[::-1]), "scans": self.scans, "cloud_points": int(len(cloud)),
                "obstacle_band_m": list(self.grid.band), "duration_s": round(traj[-1, 0]-traj[0, 0], 1) if len(traj) else 0,
                "files": {"map.yaml": "occupancy grid (ROS map_server / Nav2)", "cloud.pcd": "3-D voxel map, 0.1 m (0.05 m in height)",
                          "trajectory.csv": "SLAM poses of the LiDAR", "map.png": "preview"}}
        T = self.world_from_map() if self.evaluate else None
        if T is not None:
            meta.update(self.evaluate_against_truth(folder, grid, origin, traj, T))
        (folder/"meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
        self.get_logger().info(f"map saved to {folder} ({grid.shape[1]}x{grid.shape[0]} cells, {len(cloud)} points)")
        self.status("saved", folder=str(folder), **{k: meta[k] for k in ("trajectory", "map") if k in meta})

    def evaluate_against_truth(self, folder, grid, origin, traj, T):
        res = self.grid.resolution
        out = {"world_from_map": {"x": round(T[0, 2], 3), "y": round(T[1, 2], 3),
                                  "yaw_rad": round(math.atan2(T[1, 0], T[0, 0]), 4)}}
        truth = np.array(self.truth)
        idx = np.clip(np.searchsorted(truth[:, 0], traj[:, 0]), 0, len(truth)-1)
        tx, ty, tyaw = truth[idx, 1], truth[idx, 2], truth[idx, 3]
        lidar = np.stack([tx+np.cos(tyaw)*self.lidar_xyz[0]-np.sin(tyaw)*self.lidar_xyz[1],
                          ty+np.sin(tyaw)*self.lidar_xyz[0]+np.cos(tyaw)*self.lidar_xyz[1]], axis=1)
        slam_world = (np.c_[traj[:, 1:3], np.ones(len(traj))]@T.T)[:, :2]
        out["trajectory"] = S.trajectory_error(slam_world, lidar)
        if self.truth_map is not None:
            gt, gt_origin, gt_res = self.truth_map
            bounds = (gt_origin[0], gt_origin[1], gt_origin[0]+gt.shape[1]*gt_res, gt_origin[1]+gt.shape[0]*gt_res)
            world = S.resample(grid, origin, res, T, bounds)
            S.write_map_yaml(folder, world, gt_origin, res, name="map_world")
            out["map"] = S.compare_grids(world, gt, res)
            S.comparison_image(world, gt).save(folder/"comparison.png")
        if self.semantic_path.is_file():
            semantic = json.loads(self.semantic_path.read_text())
            (folder/"semantic_truth.json").write_text(json.dumps(
                S.transform_semantic(semantic, np.linalg.inv(T)), indent=1, ensure_ascii=False))
        (folder/"evaluation.json").write_text(json.dumps(out, indent=1))
        return out


def main():
    rclpy.init()
    node = SlamMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:  # rclpy can fail to take a message while the launch shuts the graph down
        if not node.saved:
            node.get_logger().error(f"executor stopped: {exc}")
    finally:
        try:
            node.save()
        except Exception as exc:  # never lose the shutdown because of a failed save
            node.get_logger().error(f"save failed: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
