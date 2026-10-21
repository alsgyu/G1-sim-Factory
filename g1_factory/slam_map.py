"""The robot's own map: occupancy grid and 3-D cloud from LiDAR-inertial SLAM scans (FAST-LIO2).

Frames
  camera_init  FAST-LIO2's world frame = the LiDAR/IMU frame at start-up (not levelled).
  map          what this module saves: floor at z = 0 and level, origin on the floor under the LiDAR at start-up,
               x and y along the main walls, x the wall direction closest to the robot's heading at start-up (so
               maps from the same start pose match), rooms are axis-aligned rectangles for annotation. Scans are first levelled with the gravity measured at start-up;
               the final levelling fits a plane to the floor of the whole 3-D map, because a robot that sways while
               it stands up gives a gravity estimate that is off by about a degree (1 deg = 0.7 m over 40 m).
               A person annotates this map; an external agent receives it with the annotations.

Occupancy (0.1 m cells, nav_msgs/OccupancyGrid values) from the levelled voxel map (0.1 m, 0.05 m in height): a
column is occupied when it holds at least two voxels in the obstacle band (0.15-1.6 m above the floor: the G1 is
1.32 m tall), or one that lines up with occupied neighbours (thin rails); free when it holds a floor voxel, is not
occupied and a straight line from the walked path reaches it at body height (floor seen under shelves or pallets is
not free); unknown otherwise.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path

import numpy as np

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


def gravity_alignment(accel: np.ndarray) -> np.ndarray:
    """Rotation that turns the mean specific force measured at rest (pointing up) into +z, keeping heading."""
    a = np.asarray(accel, float).reshape(-1, 3).mean(axis=0)
    up = a/np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v, c = np.cross(up, z), float(up @ z)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3)+vx+vx@vx*(1/(1+c))


def floor_height(points: np.ndarray, sensor=(0.0, 0.0, 0.0), ring=(1.0, 8.0), bin_m=0.02) -> float | None:
    """Height of the floor around the sensor in levelled coordinates: the densest height below the sensor."""
    p = np.asarray(points, float)
    r = np.hypot(p[:, 0]-sensor[0], p[:, 1]-sensor[1])
    low = p[(r > ring[0]) & (r < ring[1]) & (p[:, 2] < sensor[2]-0.5), 2]
    if len(low) < 50:
        return None
    hist, edges = np.histogram(low, bins=np.arange(low.min(), low.max()+2*bin_m, bin_m))
    k = int(np.argmax(hist))
    return float((edges[k]+edges[k+1])/2)


def fit_floor_plane(points: np.ndarray, near_m=15.0) -> np.ndarray:
    """Plane z = a x + b y + c through the floor: fitted near the origin first, then grown over the whole map."""
    p = np.asarray(points, float)
    A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
    sel = (np.hypot(p[:, 0], p[:, 1]) < near_m) & (np.abs(p[:, 2]) < 0.4)
    if sel.sum() < 50:
        sel = np.abs(p[:, 2]) < 0.4
    coef = np.linalg.lstsq(A[sel], p[sel, 2], rcond=None)[0]
    for window in (0.3, 0.2, 0.12, 0.08):
        sel = np.abs(p[:, 2]-A@coef) < window
        coef = np.linalg.lstsq(A[sel], p[sel, 2], rcond=None)[0]
    return coef


def floor_levelling(coef) -> tuple[np.ndarray, float]:
    """(R, dz) with p' = R p - (0, 0, dz) putting the plane z = a x + b y + c at z' = 0, heading kept."""
    a, b, c = coef
    R = gravity_alignment(np.array([-a, -b, 1.0]))
    return R, float((R@np.array([0.0, 0.0, c]))[2])


def visible_cells(blocked: np.ndarray, origin, res, viewpoints, max_range=25.0, n_rays=1080) -> np.ndarray:
    """Cells a straight 2-D line from any viewpoint reaches without crossing a blocked cell (body-height view)."""
    h, w = blocked.shape
    seen = np.zeros_like(blocked, bool)
    ang = np.linspace(-np.pi, np.pi, n_rays, endpoint=False)
    steps = np.arange(0.0, max_range, res*0.7)
    dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    for x, y in np.asarray(viewpoints, float):
        px = x+dirs[:, :1]*steps  # (rays, steps)
        py = y+dirs[:, 1:]*steps
        c = np.floor((px-origin[0])/res).astype(int)
        r = np.floor((py-origin[1])/res).astype(int)
        inside = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        hit = np.zeros_like(inside)
        hit[inside] = blocked[r[inside], c[inside]]
        stop = ~inside | hit
        first = np.where(stop.any(axis=1), stop.argmax(axis=1), len(steps))  # cells before the first stop
        keep = np.arange(len(steps))[None, :] < first[:, None]
        seen[r[keep], c[keep]] = True
    return seen


def wall_yaw(xy: np.ndarray, res=0.1, coarse_deg=0.5, fine_deg=0.05) -> float:
    """Yaw (rad, -45..45 deg) to rotate points by -yaw so the dominant walls lie along x and y.

    Rotated walls spread over many x and y bins; aligned walls pile into a few, so the sum of squared bin counts of
    both projections peaks at the wall direction. 0 when nothing clearly beats the current orientation.
    """
    xy = np.asarray(xy, float)
    if len(xy) < 200:
        return 0.0
    if len(xy) > 200000:
        xy = xy[np.random.default_rng(0).choice(len(xy), 200000, replace=False)]
    xy = xy-xy.mean(axis=0)

    def score(a):
        c, s = math.cos(a), math.sin(a)
        total = 0.0
        for v in (xy[:, 0]*c+xy[:, 1]*s, -xy[:, 0]*s+xy[:, 1]*c):
            counts = np.bincount(((v-v.min())/res).astype(int)).astype(float)
            total += float((counts**2).sum())
        return total
    best = max(np.radians(np.arange(-45.0, 45.0, coarse_deg)), key=score)
    best = max(best+np.radians(np.arange(-coarse_deg, coarse_deg+fine_deg/2, fine_deg)), key=score)
    return float(best) if score(best) > 1.05*score(0.0) else 0.0


@dataclass
class SlamGrid:
    """Voxel map of all scans (0.1 m, 0.05 m in height) and the occupancy grid made from it."""
    resolution: float = 0.1
    z_resolution: float = 0.05
    band: tuple = (0.15, 1.6)
    floor_tol: float = 0.08
    min_hits: int = 2
    align_walls: bool = True   # rotate the map so the walls run along x and y (any start heading)
    _chunks: list = field(init=False, default_factory=list)
    _keys: np.ndarray = field(init=False, default_factory=lambda: np.empty(0, np.int64))

    @property
    def voxel(self):
        return np.array([self.resolution, self.resolution, self.z_resolution])

    def add(self, points: np.ndarray):
        """One scan, roughly levelled, floor near z = 0 (N, 3)."""
        keys = np.floor(np.asarray(points, float)/self.voxel).astype(np.int64)+(1 << 20)
        self._chunks.append(np.unique((keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]))
        if len(self._chunks) >= 50:
            self._compact()

    def _compact(self):
        self._keys = np.unique(np.concatenate([self._keys, *self._chunks]))
        self._chunks = []

    def raw_cloud(self) -> np.ndarray:
        """Voxel centres as added (M, 3)."""
        self._compact()
        k = self._keys
        ijk = np.stack([(k >> 42) & ((1 << 21)-1), (k >> 21) & ((1 << 21)-1), k & ((1 << 21)-1)], axis=1)-(1 << 20)
        return (ijk+0.5)*self.voxel

    def build(self, margin_m=1.0, viewpoints=None, heading=None) -> dict:
        """Levelled cloud, levelling (R, dz) and the occupancy grid cropped to the observed area.

        viewpoints: LiDAR positions along the walked path (levelled frame). Floor seen under a shelf or a pallet does
        not mean the robot fits there, so floor cells count as free only when visible at body height from the path.
        heading: the robot's heading at start (levelled frame); of the four wall-aligned orientations the one closest
        to it is used, so maps made from the same start pose come out the same way round.
        """
        raw = self.raw_cloud()
        if len(raw) < 100:
            return {"grid": np.full((1, 1), UNKNOWN, np.int8), "origin": (0.0, 0.0), "cloud": raw.astype(np.float32),
                    "R": np.eye(3), "dz": 0.0, "wall_yaw": 0.0}
        R, dz = floor_levelling(fit_floor_plane(raw))
        cloud = raw@R.T-np.array([0.0, 0.0, dz])
        yaw = 0.0
        if self.align_walls:
            walls = cloud[(cloud[:, 2] >= self.band[0]) & (cloud[:, 2] <= self.band[1]), :2]
            yaw = wall_yaw(walls)
            if heading is not None:
                yaw += round((heading-yaw)/(math.pi/2))*(math.pi/2)
            if yaw:
                c, s = math.cos(-yaw), math.sin(-yaw)
                Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
                cloud, R = cloud@Rz.T, Rz@R
        cells = np.floor(cloud[:, :2]/self.resolution).astype(np.int64)
        m = int(round(margin_m/self.resolution))
        lo = cells.min(axis=0)-m
        w, h = (cells.max(axis=0)-lo+m+1)
        flat = (cells[:, 1]-lo[1])*w+(cells[:, 0]-lo[0])
        z = cloud[:, 2]
        free = np.bincount(flat[np.abs(z) < self.floor_tol], minlength=w*h).reshape(h, w) > 0
        hits = np.bincount(flat[(z >= self.band[0]) & (z <= self.band[1])], minlength=w*h).reshape(h, w)
        grid = np.full((h, w), UNKNOWN, np.int8)
        grid[free] = FREE
        # A thin rail is one voxel tall; keep it when its neighbours are occupied too (a line), drop lone voxels.
        some = (hits > 0).astype(np.int32)
        padded = np.pad(some, 1)
        neighbours = sum(padded[1+dr:1+dr+h, 1+dc:1+dc+w] for dr in (-1, 0, 1) for dc in (-1, 0, 1))
        occupied = (hits >= self.min_hits) | ((hits > 0) & (neighbours >= 3))
        if viewpoints is not None and len(viewpoints):
            vp = (np.c_[np.asarray(viewpoints, float)[:, :2], np.zeros(len(viewpoints))]@R.T)[:, :2]
            padded = np.pad(occupied, 1)  # one cell of margin so lines do not slip through gaps in thin outlines
            thick = sum(padded[1+dr:1+dr+h, 1+dc:1+dc+w] for dr in (-1, 0, 1) for dc in (-1, 0, 1)) > 0
            grid[~visible_cells(thick, lo*self.resolution, self.resolution, vp)] = UNKNOWN
        grid[occupied] = OCCUPIED
        return {"grid": grid, "origin": (float(lo[0]*self.resolution), float(lo[1]*self.resolution)),
                "cloud": cloud.astype(np.float32), "R": R, "dz": dz, "wall_yaw": yaw}

    def occupancy(self, margin_m=1.0, viewpoints=None, heading=None):
        """(int8 grid, origin xy). Row 0 = lowest y, as nav_msgs/OccupancyGrid."""
        out = self.build(margin_m, viewpoints, heading)
        return out["grid"], out["origin"]


# ------------------------------------------------------------------ files
def write_map_yaml(folder: Path, grid: np.ndarray, origin, resolution, name="map"):
    """ROS map_server / Nav2 format: <name>.pgm (254 free, 0 occupied, 205 unknown) + <name>.yaml."""
    folder.mkdir(parents=True, exist_ok=True)
    img = np.full(grid.shape, 205, np.uint8)
    img[grid == FREE] = 254
    img[grid == OCCUPIED] = 0
    img = np.flipud(img)  # image row 0 = top = highest y
    h, w = img.shape
    (folder/f"{name}.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode()+img.tobytes())
    (folder/f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")


def read_map_yaml(path: Path):
    """(int8 grid, origin xy, resolution) from a map_server YAML + PGM."""
    import yaml
    path = Path(path)
    meta = yaml.safe_load(path.read_text())
    data = (path.parent/meta["image"]).read_bytes()
    header, offset = [], 0
    while len(header) < 4:  # P5, width, height, maxval (comments skipped)
        end = data.index(b"\n", offset)
        line = data[offset:end].split(b"#")[0].split()
        header += line
        offset = end+1
    w, h = int(header[1]), int(header[2])
    img = np.frombuffer(data[offset:offset+w*h], np.uint8).reshape(h, w)
    occ = (255-img)/255.0
    grid = np.full((h, w), UNKNOWN, np.int8)
    grid[occ > meta.get("occupied_thresh", 0.65)] = OCCUPIED
    grid[occ < meta.get("free_thresh", 0.196)] = FREE  # 205 (unknown) = 0.196: not free
    return np.flipud(grid).copy(), tuple(meta["origin"][:2]), float(meta["resolution"])


def write_pcd(path: Path, xyz: np.ndarray):
    xyz = np.ascontiguousarray(xyz, np.float32)
    head = (f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            f"COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n")
    Path(path).write_bytes(head.encode()+xyz.tobytes())


def preview(grid: np.ndarray, trajectory_cells=None, scale=2):
    """RGB preview: free white, occupied dark, unknown grey, trajectory in orange."""
    from PIL import Image, ImageDraw
    rgb = np.full(grid.shape+(3,), (205, 208, 212), np.uint8)
    rgb[grid == FREE] = (255, 255, 255)
    rgb[grid == OCCUPIED] = (40, 44, 52)
    img = Image.fromarray(np.flipud(rgb)).resize((grid.shape[1]*scale, grid.shape[0]*scale), Image.NEAREST)
    if trajectory_cells is not None and len(trajectory_cells) > 1:
        h = grid.shape[0]
        pts = [((c+0.5)*scale, (h-r-0.5)*scale) for r, c in trajectory_cells]
        ImageDraw.Draw(img).line(pts, fill=(235, 110, 20), width=max(2, scale))
    return img


# ------------------------------------------------------------------ alignment and evaluation
def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def resample(grid, origin, resolution, T_dst_src: np.ndarray, dst_bounds):
    """Nearest-neighbour copy of a grid into another 2-D frame (axis-aligned dst_bounds x0, y0, x1, y1)."""
    x0, y0, x1, y1 = dst_bounds
    w, h = int(round((x1-x0)/resolution)), int(round((y1-y0)/resolution))
    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    dst = np.stack([x0+(cols+0.5)*resolution, y0+(rows+0.5)*resolution, np.ones_like(cols, float)], axis=-1)
    src = dst@np.linalg.inv(T_dst_src).T
    c = np.floor((src[..., 0]-origin[0])/resolution).astype(int)
    r = np.floor((src[..., 1]-origin[1])/resolution).astype(int)
    ok = (r >= 0) & (r < grid.shape[0]) & (c >= 0) & (c < grid.shape[1])
    out = np.full((h, w), UNKNOWN, np.int8)
    out[ok] = grid[r[ok], c[ok]]
    return out


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    out = mask.copy()
    for dr in range(-radius, radius+1):
        for dc in range(-radius, radius+1):
            if dr*dr+dc*dc > radius*radius:
                continue
            shifted = np.zeros_like(mask)
            shifted[max(dr, 0):mask.shape[0]+min(dr, 0), max(dc, 0):mask.shape[1]+min(dc, 0)] = \
                mask[max(-dr, 0):mask.shape[0]+min(-dr, 0), max(-dc, 0):mask.shape[1]+min(-dc, 0)]
            out |= shifted
    return out


def compare_grids(slam: np.ndarray, truth: np.ndarray, resolution: float, tolerance_m=0.2) -> dict:
    """SLAM grid vs ground-truth grid in the same frame and shape.

    Obstacles: precision = SLAM-occupied cells within tolerance of a true obstacle; recall = true obstacle edges
    (occupied cells next to free space) within tolerance of a SLAM obstacle, among edges the robot could see
    (next to SLAM-known space). Free space: share of the true free area the SLAM map marks free / occupied.
    """
    r = max(1, int(round(tolerance_m/resolution)))
    s_occ, t_occ = slam == OCCUPIED, truth == OCCUPIED
    t_free = truth == FREE
    edges = t_occ & dilate(t_free, 1)
    seen = dilate(slam != UNKNOWN, r)
    precision = float((s_occ & dilate(t_occ, r)).sum()/max(s_occ.sum(), 1))
    seen_edges = edges & seen
    recall = float((seen_edges & dilate(s_occ, r)).sum()/max(seen_edges.sum(), 1))
    return {"obstacle_precision": round(precision, 3), "obstacle_recall": round(recall, 3),
            "free_marked_free": round(float((t_free & (slam == FREE)).sum()/max(t_free.sum(), 1)), 3),
            "free_marked_occupied": round(float((t_free & s_occ).sum()/max(t_free.sum(), 1)), 3),
            "free_unknown": round(float((t_free & (slam == UNKNOWN)).sum()/max(t_free.sum(), 1)), 3),
            "outside_marked_free": round(float(((truth == UNKNOWN) & (slam == FREE)).sum()*resolution**2), 1),
            "tolerance_m": tolerance_m}


def comparison_image(slam: np.ndarray, truth: np.ndarray, scale=2):
    """Both obstacle (dark), SLAM only (red), ground truth only (blue), SLAM free (white), else grey."""
    from PIL import Image
    rgb = np.full(slam.shape+(3,), (215, 218, 222), np.uint8)
    rgb[slam == FREE] = (255, 255, 255)
    s, t = slam == OCCUPIED, truth == OCCUPIED
    rgb[t & ~s] = (90, 140, 220)
    rgb[s & ~t] = (220, 60, 50)
    rgb[s & t] = (35, 38, 45)
    return Image.fromarray(np.flipud(rgb)).resize((slam.shape[1]*scale, slam.shape[0]*scale), Image.NEAREST)


def trajectory_error(slam_xy: np.ndarray, truth_xy: np.ndarray) -> dict:
    """Position error of time-matched samples (already in the same frame)."""
    d = np.linalg.norm(np.asarray(slam_xy)-np.asarray(truth_xy), axis=1)
    steps = np.linalg.norm(np.diff(truth_xy, axis=0), axis=1)
    return {"ate_rmse_m": round(float(np.sqrt(np.mean(d**2))), 3), "max_m": round(float(d.max()), 3),
            "final_m": round(float(d[-1]), 3), "path_m": round(float(steps.sum()), 1),
            "final_drift_pct": round(float(100*d[-1]/max(steps.sum(), 1e-6)), 2)}


def transform_semantic(semantic: dict, T: np.ndarray) -> dict:
    """Semantic map with every point moved by the 2-D transform T (e.g. world -> map, or map -> world)."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    pt = lambda x, y: (T@np.array([x, y, 1.0]))[:2].round(3).tolist()

    def box(b):
        corners = np.array([pt(b[0], b[1]), pt(b[2], b[1]), pt(b[2], b[3]), pt(b[0], b[3])])
        return [*corners.min(axis=0).round(3).tolist(), *corners.max(axis=0).round(3).tolist()]
    out = json.loads(json.dumps(semantic))
    out["frame"] = "map (robot SLAM map: origin = floor under the LiDAR at start-up, x = start heading)"
    for z in out.get("zones", []):
        z["bounds"] = box(z["bounds"])
        z["rects"] = [box(r) for r in z.get("rects", [])]
        for key in ("entry", "entrance", "center"):
            if z.get(key):
                z[key] = pt(*z[key][:2])
    for o in out.get("objects", []):
        o["center"] = [*pt(*o["center"][:2]), o["center"][2]]
        a = o["aabb"]
        lo_hi = box([a[0], a[1], a[3], a[4]])
        o["aabb"] = [lo_hi[0], lo_hi[1], a[2], lo_hi[2], lo_hi[3], a[5]]
    for p in out.get("places", []):
        if p.get("center"):
            p["center"] = pt(*p["center"][:2])
        if p.get("bounds"):
            p["bounds"] = box(p["bounds"])
        p["rects"] = [box(r) for r in p.get("rects", [])]
    for e in out.get("edges", []):
        e["door"] = pt(*e["door"][:2])
    for a in out.get("anchors", []):
        a["xy"] = pt(*a["xy"][:2])
        if "yaw" in a:
            a["yaw"] = round(a["yaw"]+yaw, 4)
    if "aisle_segments" in out:
        out["aisle_segments"] = {k: box(v) for k, v in out["aisle_segments"].items() if v}
    out.pop("occupancy", None)
    return out
