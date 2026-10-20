"""Localisation in a saved map package: where the robot stands in the map frame of maps/<name>/.

A robot that comes back to a map it built earlier does not know where it stands in it. Its odometry (FAST-LIO2, the
G1's own estimate, or the simulator) gives a smooth pose in an odom frame of its own that drifts slowly. This module
finds map <- odom by matching LiDAR scans against the package's occupancy grid; the ROS node publishes it as the TF
map -> odom, the way AMCL and other ROS localisers do, so everything that plans in the map frame stays put.

Matching (2-D, on map.pgm)
  The map's obstacle cells are columns where the robot's SLAM saw points 0.15-1.6 m above the floor. A live scan is
  reduced the same way: points 0.2-1.5 m above the floor seen in that scan, one point per 0.1 m cell. Each point
  scores exp(-d^2 / 2 sigma^2), d = its distance to the nearest obstacle cell (likelihood field, as AMCL).
  Correlative search (Olson 2009): every rotation and translation in a window around the guess is scored, so a rough
  initial pose (1.5 m, 20 deg) is enough and no local minimum is trusted; the best peaks are then refined with finer
  steps. While tracking, the window is small and a weak prior on the odometry prediction keeps the estimate where the
  scan cannot tell (along a bare corridor).
  The odom frame must be gravity-aligned (z up); the floor seen in the scan gives the height offset (map floor z = 0).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from g1_factory import slam_map as S


def wrap(a):
    return (np.asarray(a)+np.pi) % (2*np.pi)-np.pi


def compose(a, b):
    """SE(2) a * b for (x, y, yaw) tuples."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], float(wrap(a[2]+b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


def _shift(a, dr, dc, fill):
    """out[r, c] = a[r + dr, c + dc], `fill` outside."""
    h, w = a.shape
    out = np.full_like(a, fill)
    out[max(-dr, 0):h-max(dr, 0), max(-dc, 0):w-max(dc, 0)] = a[max(dr, 0):h-max(-dr, 0), max(dc, 0):w-max(-dc, 0)]
    return out


def distance_field(occupied: np.ndarray, resolution: float, max_m: float) -> np.ndarray:
    """Distance (m) from every cell to the nearest occupied cell, capped at max_m.

    Exact Euclidean distance within the cap: the squared distance separates into a vertical pass (nearest occupied
    row in each column) and a horizontal min-plus pass over it, each a few dozen array shifts.
    """
    R = int(math.ceil(max_m/resolution))
    big = np.float32(8*(R+1)**2)
    g = np.full(occupied.shape, big, np.float32)
    for dr in range(-R, R+1):
        np.minimum(g, np.where(_shift(occupied, dr, 0, False), np.float32(dr*dr), big), out=g)
    f = np.full(occupied.shape, big, np.float32)
    for dc in range(-R, R+1):
        np.minimum(f, _shift(g, 0, dc, big)+np.float32(dc*dc), out=f)
    return np.minimum(np.sqrt(f)*resolution, max_m).astype(np.float32)


@dataclass
class Match:
    """Best pose of the scan in the map: p_map = R(yaw) (p_odom - pivot) + (x, y)."""
    x: float
    y: float
    yaw: float
    score: float          # mean point likelihood, 0..1
    inliers: float        # share of points within inlier_m of a map obstacle
    ambiguity: float      # score of the best clearly different pose / best score (1 = two equally good poses)
    cov: np.ndarray       # 3x3 spread of the near-best candidates (x, y, yaw)
    peaks: list           # refined peaks (x, y, yaw, score), best first


class ScanMatcher:
    """Scores 2-D points against the obstacle cells of an occupancy grid (likelihood field)."""

    def __init__(self, grid, origin, resolution, sigma_m=0.1, coarse_sigma_m=0.3, max_dist_m=1.0, inlier_m=0.2):
        self.origin = np.asarray(origin, float)
        self.res = float(resolution)
        self.shape = grid.shape
        self.inlier_m = inlier_m
        dist = distance_field(grid == S.OCCUPIED, self.res, max_dist_m)
        self.dist = np.pad(dist, 1, constant_values=max_dist_m)  # one cell of border for lookups outside the map
        self.fields = {}
        for name, sigma in (("fine", sigma_m), ("coarse", coarse_sigma_m)):
            f = np.exp(-0.5*(dist/sigma)**2).astype(np.float32)
            f[dist >= max_dist_m] = 0.0
            self.fields[name] = np.pad(f, 1)

    def _cells(self, x, y):
        h, w = self.shape
        c = np.clip(np.floor((x-self.origin[0])/self.res).astype(np.int64), -1, w)+1
        r = np.clip(np.floor((y-self.origin[1])/self.res).astype(np.int64), -1, h)+1
        return r, c

    def lookup(self, table, x, y, bilinear=False):
        if not bilinear:
            r, c = self._cells(x, y)
            return table[r, c]
        h, w = self.shape
        u = (x-self.origin[0])/self.res-0.5  # cell centres at integer u, v
        v = (y-self.origin[1])/self.res-0.5
        c0, r0 = np.floor(u), np.floor(v)
        fu, fv = (u-c0).astype(np.float32), (v-r0).astype(np.float32)
        c0 = c0.astype(np.int64)
        r0 = r0.astype(np.int64)
        ca, cb = np.clip(c0, -1, w)+1, np.clip(c0+1, -1, w)+1
        ra, rb = np.clip(r0, -1, h)+1, np.clip(r0+1, -1, h)+1
        top = table[ra, ca]*(1-fu)+table[ra, cb]*fu
        bottom = table[rb, ca]*(1-fu)+table[rb, cb]*fu
        return top*(1-fv)+bottom*fv

    def inliers(self, xy: np.ndarray) -> float:
        r, c = self._cells(xy[:, 0], xy[:, 1])
        return float(np.mean(self.dist[r, c] <= self.inlier_m)) if len(xy) else 0.0

    def grid_search(self, rel, center, yaw0, xy_half, xy_step, yaw_half, yaw_step, field="fine", bilinear=False,
                    prior=None):
        """Scores of all poses in a window: (scores[yaw, translation] with the prior, scan-only scores, yaws, xs, ys)."""
        n, m = int(round(xy_half/xy_step)), int(round(yaw_half/yaw_step))
        offsets = np.arange(-n, n+1)*xy_step
        tx, ty = (v.ravel() for v in np.meshgrid(center[0]+offsets, center[1]+offsets))
        yaws = yaw0+np.arange(-m, m+1)*yaw_step
        table = self.fields[field]
        scores = np.empty((len(yaws), len(tx)), np.float32)
        for k, yaw in enumerate(yaws):
            c, s = math.cos(yaw), math.sin(yaw)
            px = rel[:, 0]*c-rel[:, 1]*s
            py = rel[:, 0]*s+rel[:, 1]*c
            scores[k] = self.lookup(table, px[None, :]+tx[:, None], py[None, :]+ty[:, None], bilinear).mean(axis=1)
        raw = scores
        if prior is not None:  # (x, y, yaw, sigma_xy, sigma_yaw): odometry prediction
            px_, py_, pyaw, sxy, syaw = prior
            d2 = ((tx-px_)**2+(ty-py_)**2)/sxy**2
            scores = raw*np.exp(-0.5*(d2[None, :]+(wrap(yaws-pyaw)**2)[:, None]/syaw**2)).astype(np.float32)
        return scores, raw, yaws, tx, ty

    def match(self, points, pivot, guess, xy_window, yaw_window, prior_sigma=None, peaks=3) -> Match:
        """Best pose of `points` (N, 2, odom frame) around `guess` = (x, y, yaw) of the pivot in the map."""
        rel = np.asarray(points, float)-np.asarray(pivot, float)[:2]
        prior = None if prior_sigma is None else (*guess, *prior_sigma)
        xy_step = max(self.res, xy_window/12)
        yaw_step = max(math.radians(0.5), yaw_window/12)
        field = "coarse" if xy_window > 0.6 else "fine"
        scores, raw, yaws, tx, ty = self.grid_search(rel, guess[:2], guess[2], xy_window, xy_step, yaw_window,
                                                     yaw_step, field, prior=prior)
        # Refine the best clearly different peaks: the coarse window can rank a wrong peak first by a little.
        order = np.argsort(scores, axis=None)[::-1]
        starts = []
        for flat in order[:2000]:
            k, j = np.unravel_index(flat, scores.shape)
            cand = (tx[j], ty[j], yaws[k])
            if all(math.hypot(cand[0]-s[0], cand[1]-s[1]) > 0.5 or abs(wrap(cand[2]-s[2])) > math.radians(5)
                   for s in starts):
                starts.append(cand)
            if len(starts) == peaks:
                break
        refined = []
        for best in starts:
            step, ystep = xy_step, yaw_step
            while step > 0.01+1e-9:
                step, ystep, half, yhalf = max(step/3, 0.01), max(ystep/3, math.radians(0.1)), step, ystep
                sc, _, ys, xs, yy = self.grid_search(rel, best[:2], best[2], half, step, yhalf, ystep, "fine",
                                                     bilinear=True, prior=prior)
                k, j = np.unravel_index(int(np.argmax(sc)), sc.shape)
                best = (float(xs[j]), float(yy[j]), float(ys[k]), float(sc[k, j]))
            refined.append(best)
        refined.sort(key=lambda r: -r[3])
        x, y, yaw, score = refined[0]
        # Peaks that refined back into the best one's basin are the same pose, not an alternative.
        others = [r[3] for r in refined[1:] if math.hypot(r[0]-x, r[1]-y) > 0.5 or abs(wrap(r[2]-yaw)) > math.radians(5)]
        ambiguity = max(others)/score if others and score > 0 else 0.0
        # Spread of the window poses the scan alone rates nearly as good as its best: wide along a bare corridor
        # (where the odometry prior decides), small at a corner.
        k, j = np.nonzero(raw >= 0.95*raw.max())
        d = np.c_[tx[j]-x, ty[j]-y, wrap(yaws[k]-yaw)]
        cov = d.T@d/len(d)+np.diag([xy_step**2, xy_step**2, yaw_step**2])/12
        c, s = math.cos(yaw), math.sin(yaw)
        placed = rel@np.array([[c, s], [-s, c]])+np.array([x, y])
        fit = float(self.lookup(self.fields["fine"], placed[:, 0], placed[:, 1], bilinear=True).mean())
        return Match(x, y, float(wrap(yaw)), fit, self.inliers(placed), float(ambiguity), cov, refined)


@dataclass
class Update:
    state: str                          # localized, rejected, waiting
    message: str = ""
    map_from_odom: tuple | None = None  # (x, y, yaw)
    z: float = 0.0                      # map z = odom z + z (floor seen in the scan)
    match: Match | None = None
    points: int = 0
    correction_m: float = 0.0           # how far the estimate of the LiDAR position moved
    correction_deg: float = 0.0
    initializing: bool = False


@dataclass
class Localizer:
    """map <- odom from LiDAR scans and a rough initial pose; ROS-free (g1_factory.ros2.localizer_node runs it)."""
    grid: np.ndarray
    origin: tuple
    resolution: float
    band: tuple = (0.2, 1.5)                    # scan points used: height above the floor (map: 0.15-1.6 m)
    range_m: tuple = (0.6, 25.0)                # the robot's own body is within 0.6 m of the head LiDAR
    cell_m: float = 0.1
    max_points: int = 800
    init_window: tuple = (1.5, math.radians(20))
    track_window: tuple = (0.4, math.radians(5))
    prior: tuple = (0.5, math.radians(8))       # odometry prediction while tracking
    min_inliers: float = 0.5
    init_min_inliers: float = 0.8               # first fix: a wrong pose still puts 60-70 % of points on obstacles
    max_ambiguity: float = 0.97                 # first fix: refuse two equally good poses
    lost_after: int = 6                         # rejected updates in a row before searching the initial window again
    seed: int = 0
    T: tuple | None = field(init=False, default=None)
    z: float = field(init=False, default=0.0)
    initializing: bool = field(init=False, default=False)
    failures: int = field(init=False, default=0)

    def __post_init__(self):
        self.matcher = ScanMatcher(self.grid, self.origin, self.resolution)
        self.rng = np.random.default_rng(self.seed)

    @property
    def localized(self):
        return self.T is not None and not self.initializing

    def initialize(self, map_pose, odom_pose, window=None):
        """Rough pose of base_link in the map (a person's guess) and its odom pose at the same moment."""
        self.T = compose(tuple(map_pose), inverse(tuple(odom_pose)))
        self.initializing, self.failures = True, 0
        if window is not None:
            self.init_window = (float(window[0]), float(window[1]))

    def scan_points(self, xyz, sensor):
        """2-D obstacle points of one scan in odom (one per cell) and the floor height, or (None, None)."""
        xyz = np.asarray(xyz, float)
        floor = S.floor_height(xyz, sensor)
        if floor is None:
            return None, None
        h = xyz[:, 2]-floor
        r = np.hypot(xyz[:, 0]-sensor[0], xyz[:, 1]-sensor[1])
        pts = xyz[(h >= self.band[0]) & (h <= self.band[1]) & (r >= self.range_m[0]) & (r <= self.range_m[1]), :2]
        _, first = np.unique(np.floor(pts/self.cell_m).astype(np.int64), axis=0, return_index=True)
        pts = pts[np.sort(first)]
        if len(pts) > self.max_points:
            pts = pts[np.sort(self.rng.choice(len(pts), self.max_points, replace=False))]
        return pts, floor

    def update(self, xyz_odom, sensor_odom) -> Update:
        """One scan (N, 3) in the odom frame and the LiDAR position in odom."""
        if self.T is None:
            return Update("waiting", "no initial pose yet")
        pts, floor = self.scan_points(xyz_odom, sensor_odom)
        if pts is None or len(pts) < 30:
            return self._reject("too few obstacle points in the scan", 0 if pts is None else len(pts))
        searching = self.initializing or self.failures >= self.lost_after
        window = self.init_window if searching else self.track_window
        guess = (*compose(self.T, (sensor_odom[0], sensor_odom[1], 0.0))[:2], self.T[2])
        m = self.matcher.match(pts, sensor_odom[:2], guess, window[0], window[1],
                               prior_sigma=(max(window[0], self.prior[0]), max(window[1], self.prior[1]))
                               if searching else self.prior, peaks=3 if searching else 1)
        need = self.init_min_inliers if searching else self.min_inliers
        if m.inliers < need:
            return self._reject(f"scan fits the map poorly ({m.inliers:.0%} of points on map obstacles)", len(pts), m)
        if searching and m.ambiguity > self.max_ambiguity:
            return self._reject(f"two poses fit equally well (ambiguity {m.ambiguity:.2f})", len(pts), m)
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        T = (m.x-(c*sensor_odom[0]-s*sensor_odom[1]), m.y-(s*sensor_odom[0]+c*sensor_odom[1]), m.yaw)
        moved = math.hypot(m.x-guess[0], m.y-guess[1])
        turned = math.degrees(abs(float(wrap(m.yaw-guess[2]))))
        self.z = -floor if searching else 0.8*self.z+0.2*-floor
        was = self.initializing
        self.T, self.initializing, self.failures = T, False, 0
        return Update("localized", map_from_odom=T, z=self.z, match=m, points=len(pts), correction_m=moved,
                      correction_deg=turned, initializing=was)

    def _reject(self, message, n, m=None):
        self.failures += 1
        return Update("rejected", message, self.T, self.z, m, n, initializing=self.initializing)

    def map_pose(self, odom_pose):
        """(x, y, yaw) of an odom-frame pose in the map, or None before the first fix."""
        return compose(self.T, tuple(odom_pose)) if self.localized else None
