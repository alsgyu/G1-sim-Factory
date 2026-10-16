"""numpy <-> ROS 2 message helpers without cv_bridge (works with Humble's numpy)."""
from __future__ import annotations

from array import array
import math

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

# livox_ros_driver2 PointCloud2 layout (xfer_format=0): packed LivoxPointXyzrtlt, 26 bytes.
LIVOX_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                        ("tag", "u1"), ("line", "u1"), ("timestamp", "<f8")])
assert LIVOX_DTYPE.itemsize == 26


def stamp(seconds: float) -> Time:
    sec = math.floor(seconds)
    return Time(sec=int(sec), nanosec=int(round((seconds-sec)*1e9)) % 1_000_000_000)


def to_seconds(t) -> float:
    return t.sec+t.nanosec*1e-9


def yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))


def quaternion_yaw(q) -> float:
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def quaternion_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_matrix(transform) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    m = np.eye(4)
    m[:3, :3] = quaternion_matrix(transform.rotation)
    m[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return m


def image_msg(t: float, frame: str, pixels: np.ndarray, encoding: str) -> Image:
    pixels = np.ascontiguousarray(pixels)
    msg = Image()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding, msg.is_bigendian = encoding, 0
    msg.step = pixels.strides[0]
    msg.data = array("B", pixels.tobytes())  # array.array avoids per-element validation
    return msg


def image_to_numpy(msg: Image) -> np.ndarray:
    dtypes = {"rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3), "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
              "mono8": (np.uint8, 1), "16UC1": (np.uint16, 1), "mono16": (np.uint16, 1), "32FC1": (np.float32, 1)}
    if msg.encoding not in dtypes:
        raise ValueError(f"Unsupported image encoding {msg.encoding}")
    dtype, channels = dtypes[msg.encoding]
    item = np.dtype(dtype).itemsize*channels
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*item]
    image = raw.copy().view(dtype).reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        image = image[..., [2, 1, 0]+([3] if channels == 4 else [])]
    return image[..., 0] if channels == 1 else image[..., :3]


def depth_to_metres(msg: Image) -> np.ndarray:
    """16UC1 millimetres (RealSense) or 32FC1 metres (Isaac Sim); 0/NaN = no return."""
    depth = image_to_numpy(msg).astype(np.float32)
    if msg.encoding in ("16UC1", "mono16"):
        depth *= 0.001
    depth[~np.isfinite(depth)] = 0
    return depth


def camera_info_msg(t, frame, width, height, fx, fy, cx, cy) -> CameraInfo:
    msg = CameraInfo()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    msg.width, msg.height = int(width), int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0]*5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def livox_cloud_msg(t, frame, xyz, intensity, line, point_time_ns) -> PointCloud2:
    points = np.zeros(len(xyz), dtype=LIVOX_DTYPE)
    points["x"], points["y"], points["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    points["intensity"], points["line"], points["timestamp"] = intensity, line, point_time_ns
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    msg.height, msg.width = 1, len(points)
    names = [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
             ("intensity", 12, PointField.FLOAT32), ("tag", 16, PointField.UINT8),
             ("line", 17, PointField.UINT8), ("timestamp", 18, PointField.FLOAT64)]
    msg.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in names]
    msg.is_bigendian, msg.point_step = False, LIVOX_DTYPE.itemsize
    msg.row_step, msg.is_dense = msg.point_step*msg.width, True
    msg.data = array("B", points.tobytes())
    return msg


_FIELD_DTYPES = {PointField.FLOAT32: "<f4", PointField.FLOAT64: "<f8", PointField.UINT8: "u1",
                 PointField.INT8: "i1", PointField.UINT16: "<u2", PointField.INT16: "<i2",
                 PointField.UINT32: "<u4", PointField.INT32: "<i4"}


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    """(N, 3) float array from any PointCloud2 with x/y/z fields (Livox, Isaac RTX LiDAR...)."""
    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"} <= set(fields):
        raise ValueError("PointCloud2 needs x, y, z fields")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n = msg.width*msg.height
    raw = raw[:n*msg.point_step].reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float64)
    for k, name in enumerate("xyz"):
        f = fields[name]
        dtype = np.dtype(_FIELD_DTYPES[f.datatype])
        out[:, k] = raw[:, f.offset:f.offset+dtype.itemsize].copy().view(dtype).ravel()
    return out[np.isfinite(out).all(axis=1)]


def occupancy_msg(t, frame, origin, resolution, data: np.ndarray) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    msg.info.map_load_time = stamp(t)
    msg.info.resolution = float(resolution)
    msg.info.height, msg.info.width = data.shape
    msg.info.origin.position.x, msg.info.origin.position.y = float(origin[0]), float(origin[1])
    msg.info.origin.orientation.w = 1.0
    msg.data = array("b", np.ascontiguousarray(data, dtype=np.int8).tobytes())
    return msg


def occupancy_to_numpy(msg: OccupancyGrid) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width).copy()


def pose_stamped(t, frame, x, y, yaw) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation = yaw_quaternion(0.0 if yaw is None else yaw)
    return msg


def path_msg(t, frame, points, final_yaw=None) -> Path:
    msg = Path()
    msg.header.stamp, msg.header.frame_id = stamp(t), frame
    points = np.asarray(points, dtype=float)
    for k, (x, y) in enumerate(points):
        if k+1 < len(points):
            yaw = math.atan2(points[k+1, 1]-y, points[k+1, 0]-x)
        else:
            yaw = final_yaw if final_yaw is not None else (math.atan2(y-points[k-1, 1], x-points[k-1, 0]) if k else 0.0)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation = yaw_quaternion(yaw)
        msg.poses.append(pose)
    return msg


def path_to_numpy(msg: Path):
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float).reshape(-1, 2)
    final_yaw = quaternion_yaw(msg.poses[-1].pose.orientation) if msg.poses else None
    return pts, final_yaw


def pose(x, y, yaw) -> Pose:
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation = yaw_quaternion(0.0 if yaw is None else yaw)
    return p
