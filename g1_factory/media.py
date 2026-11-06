"""Still images and navigation videos from Isaac Sim: where the cameras stand and where the frames go.

g1_factory/isaac/isaac_sim_node.py renders (--overview, --video); this module has no Isaac imports, so the camera
poses and the recorder are tested without a GPU (tests/test_media.py). scripts/make_videos.py turns the recorded
frames into MP4 files.

Camera rotations use Isaac's "world" camera axes: columns are the camera's forward (x), left (y) and up (z) in the
world frame, the convention isaac_sim_node.make_camera and Camera.set_world_pose(camera_axes="world") take.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, wait
import json
import math
from pathlib import Path
import struct
import zlib

import numpy as np

# Still views of the campus: (region, azimuth of the viewing direction, elevation below the horizon, horizontal FOV).
# Regions come from campus_meta.json; azimuth 90 deg looks north (+y), so the warehouse is on the left, the office on
# the right and the pump room south of the office in the foreground, as in docs/images/campus.png.
VIEWS = {
    "campus": ("campus", 90.0, 58.0, 45.0),
    "top": ("campus", 90.0, 90.0, 30.0),
    "warehouse": ("warehouse_bounds", 90.0, 58.0, 45.0),
    "office": ("office_bounds", 90.0, 58.0, 45.0),
    "pump_room": ("utility_bounds", 60.0, 55.0, 45.0),
}
WALL_TOP_M = 6.0  # tallest walls (warehouse); the fit keeps their tops in the frame


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """3x3 rotation with columns forward, left, up. Looking straight down, north (+y) is up in the image."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    forward = target-eye
    forward /= np.linalg.norm(forward)
    left = np.cross(np.asarray(up, float), forward)
    if np.linalg.norm(left) < 1e-6:
        left = np.cross([0.0, 1.0, 0.0], forward)
    left /= np.linalg.norm(left)
    return np.column_stack([forward, left, np.cross(forward, left)])


def view_bounds(meta: dict, region: str) -> list[float]:
    """[xmin, ymin, xmax, ymax] of a campus region; "campus" is every building together."""
    if region != "campus":
        return list(meta[region])
    boxes = [meta[k] for k in ("warehouse_bounds", "office_bounds", "utility_bounds") if k in meta]
    return [min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes)]


def project(points, eye, rotation, hfov, aspect):
    """Image coordinates of world points: u right, v up, both in [-1, 1] inside the frame; depth along the view."""
    c = (np.asarray(points, float)-eye)@rotation
    depth = c[:, 0]
    tan_h = math.tan(hfov/2)
    return -c[:, 1]/depth/tan_h, c[:, 2]/depth/(tan_h/aspect), depth


def fit_view(bounds, azimuth_deg, elevation_deg, hfov_deg, aspect, z_top=WALL_TOP_M, margin=0.04):
    """Eye and target of a perspective camera that frames the box bounds x [0, z_top], centred, at the given angles.

    Bisection on the distance finds the closest camera that keeps every box corner inside 1-margin of the frame;
    the target then moves so the projected box sits in the middle (a tilted view draws the near side larger)."""
    x0, y0, x1, y1 = bounds
    corners = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (0.0, z_top)])
    az, el, hfov = math.radians(azimuth_deg), math.radians(elevation_deg), math.radians(hfov_deg)
    forward = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), -math.sin(el)])
    target = np.array([(x0+x1)/2, (y0+y1)/2, 0.0])
    tan_h = math.tan(hfov/2)
    tan_v = tan_h/aspect

    def place(distance):
        eye = target-distance*forward
        rotation = look_at(eye, target)
        u, v, depth = project(corners, eye, rotation, hfov, aspect)
        return eye, rotation, u, v, depth

    for _ in range(5):
        near, far = 0.1, 1e4
        for _ in range(80):
            mid = math.sqrt(near*far)
            *_, u, v, depth = place(mid)
            fits = depth.min() > 0 and max(np.abs(u).max(), np.abs(v).max()) <= 1-margin
            near, far = (near, mid) if fits else (mid, far)
        eye, rotation, u, v, _ = place(far)
        du, dv = (u.max()+u.min())/2, (v.max()+v.min())/2
        depth_target = float(np.dot(target-eye, forward))
        target = target-rotation[:, 1]*du*tan_h*depth_target+rotation[:, 2]*dv*tan_v*depth_target
    return place(far)[0], target


class ChaseCamera:
    """Third-person camera behind and above the robot, smoothed so its walking sway does not shake the image.

    The default stands higher than the office (3 m) and pump room (4 m) walls and looks down about 50 degrees, so a
    wall right behind the robot hides little of it after a doorway."""

    def __init__(self, back=2.8, height=4.6, ahead=1.5, target_dz=-0.4, smoothing=0.85):
        self.back, self.height, self.ahead, self.target_dz, self.smoothing = back, height, ahead, target_dz, smoothing
        self.eye = self.target = None

    def update(self, position, rotation):
        """position: base (pelvis) in the world; rotation: its 3x3 orientation. Returns the smoothed eye, target."""
        rotation = np.asarray(rotation, float)
        forward = np.array([rotation[0, 0], rotation[1, 0], 0.0])
        forward /= max(np.linalg.norm(forward), 1e-6)
        position = np.asarray(position, float)
        eye = position-self.back*forward+np.array([0.0, 0.0, self.height])
        target = position+self.ahead*forward+np.array([0.0, 0.0, self.target_dz])
        if self.eye is not None:
            keep = self.smoothing
            eye, target = keep*self.eye+(1-keep)*eye, keep*self.target+(1-keep)*target
        self.eye, self.target = eye, target
        return eye, target


def as_rgb8(image) -> np.ndarray | None:
    """H x W x 3 uint8 from an Isaac get_rgba() result (uint8 or float in [0, 1]); None while nothing is rendered."""
    if image is None:
        return None
    image = np.asarray(image)
    if image.ndim != 3 or image.size == 0:
        return None
    if image.dtype != np.uint8:
        image = (np.clip(image, 0.0, 1.0)*255+0.5).astype(np.uint8)
    return np.ascontiguousarray(image[..., :3])


def write_png(path: Path, rgb: np.ndarray, level: int = 3) -> Path:
    """Plain RGB PNG with numpy and zlib only (Isaac Sim's Python does not always have Pillow)."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = rgb.shape[:2]
    rows = np.zeros((height, 1+3*width), np.uint8)  # filter byte 0 (none) in front of every row
    rows[:, 1:] = rgb.reshape(height, 3*width)

    def chunk(tag, data):
        return struct.pack(">I", len(data))+tag+data+struct.pack(">I", zlib.crc32(tag+data) & 0xFFFFFFFF)

    Path(path).write_bytes(b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(rows.tobytes(), level))+chunk(b"IEND", b""))
    return Path(path)


def have_pillow() -> bool:
    try:
        import PIL.Image  # noqa: F401
        return True
    except ImportError:
        return False


def save_image(path: Path, rgb: np.ndarray, quality: int = 90) -> Path:
    """By suffix: .jpg (Pillow) or .png (Pillow, else write_png)."""
    path = Path(path)
    if have_pillow():
        from PIL import Image
        options = {"quality": quality} if path.suffix == ".jpg" else {}
        Image.fromarray(rgb).save(path, **options)
        return path
    if path.suffix == ".jpg":
        raise RuntimeError("JPEG needs Pillow; save as .png")
    return write_png(path, rgb)


class FrameRecorder:
    """First-person (head camera) and third-person (chase camera) frames of each VLN task, as numbered images.

    mode "task": a segment starts with the agent's "accepted" feedback and ends tail_s simulated seconds after
    "done", "failed" or "canceled" of that task (a new "accepted" closes the running one as "preempted").
    mode "always": one segment "run" from the first frame to the end of the simulation.
    Each segment folder holds fpv/000000.jpg ..., tpv/000000.jpg ... (the same count, frame k of both taken at the
    same simulated time) and frames.json; scripts/make_videos.py encodes them. Images are written by worker threads
    so the simulation loop only hands over arrays."""

    TERMINAL = ("done", "failed", "canceled")

    def __init__(self, out: Path, mode: str = "task", fps: float = 30.0, tail_s: float = 2.0, workers: int = 2,
                 quality: int = 90, log=print):
        if mode not in ("task", "always"):
            raise ValueError("mode must be task or always")
        self.out, self.mode, self.fps, self.tail_s, self.quality, self.log = Path(out), mode, fps, tail_s, quality, log
        self.ext = ".jpg" if have_pillow() else ".png"
        self.pool = ThreadPoolExecutor(max_workers=workers)
        self.segment = None
        self.closed = []
        self.started_always = False
        self.out.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ events
    def on_feedback(self, event: dict, t: float):
        """One /vln/feedback event (parsed JSON) at simulated time t."""
        state, task_id = event.get("state"), event.get("task_id")
        if state == "accepted" and self.mode == "task":
            if self.segment is not None:
                self.close("preempted")
            self.open(t, task_id or "task", event.get("instruction", ""))
        seg = self.segment
        if seg is None:
            return
        seg["events"].append({"sim_time_s": round(t, 3), "state": state})
        if state in self.TERMINAL and (self.mode == "always" or task_id == seg["task_id"]):
            seg["final_event"] = event
            seg["state"] = state
            if self.mode == "task" and seg["stop_at"] is None:
                seg["stop_at"] = t+self.tail_s

    def wants_frame(self, t: float) -> bool:
        """Call once per camera frame before rendering the chase view: closes a segment whose tail has passed."""
        if self.mode == "always" and not self.started_always:
            self.started_always = True
            self.open(t, "run", "")
        seg = self.segment
        if seg is not None and seg["stop_at"] is not None and t >= seg["stop_at"]:
            self.close(seg["state"])
        return self.segment is not None

    # ------------------------------------------------------------------ frames
    def add(self, t: float, fpv: np.ndarray, tpv: np.ndarray):
        seg = self.segment
        if seg is None:
            return
        k = len(seg["sim_times"])
        if k == 0:
            seg["fpv_size"], seg["tpv_size"] = [int(fpv.shape[1]), int(fpv.shape[0])], [int(tpv.shape[1]),
                                                                                          int(tpv.shape[0])]
        seg["sim_times"].append(round(t, 4))
        for view, image in (("fpv", fpv), ("tpv", tpv)):
            path = seg["dir"]/view/f"{k:06d}{self.ext}"
            seg["futures"].append(self.pool.submit(save_image, path, np.array(image, copy=True), self.quality))
        seg["futures"] = [f for f in seg["futures"] if not f.done() or f.exception() is not None]

    # ------------------------------------------------------------------ segments
    def open(self, t, task_id, instruction):
        index = 1+sum(1 for p in self.out.iterdir() if p.is_dir())  # numbering continues in a reused folder
        name = "run" if self.mode == "always" else f"{index:02d}_{task_id}"
        path = self.out/name
        k = 2
        while path.exists():
            path, k = self.out/f"{name}_{k}", k+1
        for view in ("fpv", "tpv"):
            (path/view).mkdir(parents=True, exist_ok=True)
        self.segment = {"dir": path, "task_id": task_id, "instruction": instruction, "start_sim_s": round(t, 3),
                        "stop_at": None, "state": None, "final_event": None, "events": [], "sim_times": [],
                        "futures": [], "fpv_size": None, "tpv_size": None}
        self.log(f"video: recording {path.name}" + (f" ({instruction})" if instruction else ""))

    def close(self, state=None):
        seg, self.segment = self.segment, None
        if seg is None:
            return None
        wait(seg["futures"])
        errors = [str(f.exception()) for f in seg["futures"] if f.exception() is not None]
        times = seg["sim_times"]
        info = {"instruction": seg["instruction"], "task_id": seg["task_id"], "state": state or seg["state"],
                "fps": self.fps, "ext": self.ext, "frames": len(times), "start_sim_s": seg["start_sim_s"],
                "end_sim_s": times[-1] if times else seg["start_sim_s"], "fpv_size": seg["fpv_size"],
                "tpv_size": seg["tpv_size"], "final_event": seg["final_event"], "events": seg["events"],
                "write_errors": errors[:5], "sim_times": times}
        (seg["dir"]/"frames.json").write_text(json.dumps(info, ensure_ascii=False, indent=1)+"\n")
        self.closed.append(seg["dir"])
        self.log(f"video: {seg['dir'].name} closed ({info['state']}), {len(times)} frames "
                 f"= {len(times)/self.fps:.1f} s; encode with scripts/make_videos.py")
        return seg["dir"]

    def shutdown(self):
        """End of the simulation: close a running segment as "interrupted" (or with its terminal state)."""
        if self.segment is not None:
            self.close(self.segment["state"] or ("end" if self.mode == "always" else "interrupted"))
        self.pool.shutdown(wait=True)
