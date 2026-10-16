"""Hierarchical navigation layers 2 and 3: grid planning and path tracking.

Layer 1 (language grounding) lives in ``grounding.py`` and produces map-frame
goal poses. This module turns a goal pose into a collision-free path on an
occupancy grid (A* + line-of-sight smoothing) and tracks that path with a
pure-pursuit controller that also uses the G1 policy's lateral velocity input.
Nothing here imports ROS; the ROS 2 nodes in ``g1_factory.ros2`` wrap it.
All coordinates are world/map-frame metres, yaw in radians.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

import numpy as np

from .navigation import wrap_angle

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


def _disk_offsets(radius_cells: int):
    r = int(radius_cells)
    out = []
    for dy in range(-r, r+1):
        for dx in range(-r, r+1):
            dist = math.hypot(dx, dy)
            if dist <= r:
                out.append((dy, dx, dist))
    return out


def distance_field(blocked: np.ndarray, resolution: float, max_distance: float) -> np.ndarray:
    """Metres from every cell to the nearest blocked cell, saturated at max_distance.

    Brute-force disk sweep: O(cells * disk) with vectorised numpy shifts, which
    is fast enough for the campus grid at start-up and for local windows.
    """
    h, w = blocked.shape
    out = np.full((h, w), float(max_distance), dtype=np.float32)
    if not blocked.any():
        return out
    for dy, dx, dist in _disk_offsets(math.ceil(max_distance/resolution)):
        d = dist*resolution
        if d >= max_distance:
            continue
        src = blocked[max(0, -dy):h-max(0, dy), max(0, -dx):w-max(0, dx)]
        view = out[max(0, dy):h-max(0, -dy), max(0, dx):w-max(0, -dx)]
        np.minimum(view, np.where(src, d, max_distance), out=view)
    return out


@dataclass
class OccupancyGrid:
    """Row-major grid; cell (row, col) covers origin + (col, row) * resolution."""

    origin: tuple
    resolution: float
    data: np.ndarray  # int8, FREE / OCCUPIED / UNKNOWN

    @property
    def height(self):
        return self.data.shape[0]

    @property
    def width(self):
        return self.data.shape[1]

    def world_to_cell(self, xy):
        xy = np.asarray(xy, dtype=float)
        col = np.floor((xy[..., 0]-self.origin[0])/self.resolution).astype(int)
        row = np.floor((xy[..., 1]-self.origin[1])/self.resolution).astype(int)
        return row, col

    def cell_to_world(self, row, col):
        return np.stack([self.origin[0]+(np.asarray(col)+.5)*self.resolution,
                         self.origin[1]+(np.asarray(row)+.5)*self.resolution], axis=-1)

    def inside(self, row, col):
        return (0 <= row) & (row < self.height) & (0 <= col) & (col < self.width)

    @classmethod
    def from_scene(cls, metadata: dict, resolution: float = 0.1, extra_obstacles=()):
        """Static map of the shared campus from scene metadata (simulator ground truth).

        Floor outside the warehouse, bridge and office is UNKNOWN, i.e. not
        traversable. Obstacle footprints are the scene's collision AABBs that
        reach robot height (see scene._solid / office obstacle()).
        """
        x0, y0, x1, y1 = metadata["bounds"]
        width = int(round((x1-x0)/resolution))
        height = int(round((y1-y0)/resolution))
        grid = cls((float(x0), float(y0)), float(resolution), np.full((height, width), UNKNOWN, np.int8))
        layout = metadata["config"]["layout"]
        bridge_half = layout["corridor_width"]/2+0.15
        wb, ob = layout["warehouse_bounds"], layout["office_bounds"]
        for b in (wb, ob, [wb[2]-0.2, -bridge_half, ob[0]+0.2, bridge_half]):
            grid.fill_box(b, FREE)
        for item in list(metadata.get("obstacles", []))+list(extra_obstacles):
            grid.fill_box(item["bounds"], OCCUPIED)
        return grid

    def fill_box(self, bounds, value):
        r0, c0 = self.world_to_cell(np.array([bounds[0], bounds[1]]))
        r1, c1 = self.world_to_cell(np.array([bounds[2]-1e-9, bounds[3]-1e-9]))
        r0, c0 = max(0, int(r0)), max(0, int(c0))
        r1, c1 = min(self.height-1, int(r1)), min(self.width-1, int(c1))
        if r0 <= r1 and c0 <= c1:
            self.data[r0:r1+1, c0:c1+1] = value

    def blocked(self):
        return self.data != FREE


@dataclass
class CostMap:
    """Static distance field plus a local dynamic layer from LiDAR returns."""

    grid: OccupancyGrid
    lethal_radius: float = 0.40     # robot footprint radius incl. margin
    soft_radius: float = 1.00       # keep paths centred in aisles when possible
    static_distance: np.ndarray = field(init=False)
    dynamic_hits: np.ndarray = field(init=False)

    def __post_init__(self):
        self.static_distance = distance_field(self.grid.blocked(), self.grid.resolution, self.soft_radius+0.2)
        self.dynamic_hits = np.zeros(self.grid.data.shape, dtype=np.float32)  # last hit time, 0 = none
        self.distance = self.static_distance.copy()
        self._dynamic_window = None

    # ---- dynamic obstacle layer -------------------------------------------------
    def mark_points(self, points_xy: np.ndarray, stamp: float):
        """Mark obstacle cells hit by LiDAR points (already filtered by height/range)."""
        if len(points_xy) == 0:
            return 0
        row, col = self.grid.world_to_cell(points_xy)
        ok = self.grid.inside(row, col)
        row, col = row[ok], col[ok]
        # Static obstacles already block these cells; keep only unmapped ones.
        unmapped = self.static_distance[row, col] > self.grid.resolution*1.5
        self.dynamic_hits[row[unmapped], col[unmapped]] = stamp
        return int(unmapped.sum())

    def update(self, now: float, center_xy, decay_s: float = 10.0, window_m: float = 12.0):
        """Recompute the distance field near the robot from recent dynamic hits."""
        self.dynamic_hits[(self.dynamic_hits > 0) & (now-self.dynamic_hits > decay_s)] = 0
        if self._dynamic_window is not None:
            r0, r1, c0, c1 = self._dynamic_window
            self.distance[r0:r1, c0:c1] = self.static_distance[r0:r1, c0:c1]
            self._dynamic_window = None
        if not (self.dynamic_hits > 0).any():
            return
        res = self.grid.resolution
        row, col = self.grid.world_to_cell(np.asarray(center_xy))
        half = int(window_m/res)
        r0, r1 = max(0, row-half), min(self.grid.height, row+half)
        c0, c1 = max(0, col-half), min(self.grid.width, col+half)
        local = distance_field(self.dynamic_hits[r0:r1, c0:c1] > 0, res, self.soft_radius+0.2)
        self.distance[r0:r1, c0:c1] = np.minimum(self.static_distance[r0:r1, c0:c1], local)
        self._dynamic_window = (r0, r1, c0, c1)

    # ---- queries ---------------------------------------------------------------
    def traversable(self, row, col):
        return self.grid.inside(row, col) and self.distance[row, col] >= self.lethal_radius

    def occupied(self, xy) -> bool:
        """The static (SLAM) map has an obstacle at xy; outside the map counts as occupied."""
        row, col = self.grid.world_to_cell(np.asarray(xy, dtype=float))
        return not bool(self.grid.inside(row, col)) or bool(self.grid.data[row, col] != FREE)

    def nearest_traversable(self, xy, max_radius: float = 1.5):
        """Closest traversable cell centre to xy, or None (used to snap goals)."""
        row, col = self.grid.world_to_cell(np.asarray(xy, dtype=float))
        if self.traversable(row, col):
            return np.asarray(xy, dtype=float)
        best = None
        for dy, dx, dist in sorted(_disk_offsets(int(max_radius/self.grid.resolution)), key=lambda o: o[2]):
            r, c = row+dy, col+dx
            if self.traversable(r, c):
                best = self.grid.cell_to_world(r, c)
                break
        return best

    def to_ros_costmap(self) -> np.ndarray:
        """int8 costs 0..100 for nav_msgs/OccupancyGrid (100 = lethal, -1 unknown)."""
        cost = np.clip((self.soft_radius-self.distance)/(self.soft_radius-self.lethal_radius), 0, 1)*99
        out = cost.astype(np.int8)
        out[self.distance < self.lethal_radius] = 100
        out[self.grid.data == UNKNOWN] = -1
        return out


class PlanningError(RuntimeError):
    pass


def astar(costmap: CostMap, start_xy, goal_xy, *, proximity_weight: float = 3.0,
          heuristic_weight: float = 1.0, max_expansions: int = 2_000_000):
    """8-connected A* over traversable cells. Returns an (N, 2) world-frame polyline.

    Step cost = length * (1 + proximity_weight * c), c in [0, 1] rising towards
    obstacles, so the path prefers aisle centres without forbidding narrow doors.
    """
    grid = costmap.grid
    start = costmap.nearest_traversable(start_xy, 0.8)
    goal = costmap.nearest_traversable(goal_xy, 1.5)
    if start is None:
        raise PlanningError("start pose is inside an obstacle")
    if goal is None:
        raise PlanningError("goal pose is inside or too close to an obstacle")
    sr, sc = (int(v) for v in grid.world_to_cell(start))
    gr, gc = (int(v) for v in grid.world_to_cell(goal))
    w = grid.width
    dist = costmap.distance
    soft, lethal = costmap.soft_radius, costmap.lethal_radius
    penalty = 1.0+proximity_weight*np.clip((soft-dist)/(soft-lethal), 0, 1)**2
    passable = (dist >= lethal).ravel()
    penalty = penalty.ravel()
    neighbours = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                  (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    sqrt2m2 = math.sqrt(2)-2

    def h(r, c):
        dr, dc = abs(r-gr), abs(c-gc)
        return heuristic_weight*(dr+dc+sqrt2m2*min(dr, dc))

    start_i, goal_i = sr*w+sc, gr*w+gc
    g = {start_i: 0.0}
    parent = {start_i: -1}
    heap = [(h(sr, sc), 0.0, start_i)]
    closed = set()
    expansions = 0
    H = grid.height
    while heap:
        _, cost, i = heapq.heappop(heap)
        if i in closed:
            continue
        if i == goal_i:
            break
        closed.add(i)
        expansions += 1
        if expansions > max_expansions:
            raise PlanningError("A* expansion limit reached")
        r, c = divmod(i, w)
        for dr, dc, step in neighbours:
            nr, nc = r+dr, c+dc
            if nr < 0 or nc < 0 or nr >= H or nc >= w:
                continue
            j = nr*w+nc
            if not passable[j] or j in closed:
                continue
            if dr and dc and not (passable[r*w+nc] and passable[nr*w+c]):
                continue  # no corner cutting
            new = cost+step*0.5*(penalty[i]+penalty[j])
            if new < g.get(j, math.inf):
                g[j] = new
                parent[j] = i
                heapq.heappush(heap, (new+h(nr, nc), new, j))
    else:
        raise PlanningError("no collision-free path to the goal")
    cells = []
    i = goal_i
    while i != -1:
        cells.append(divmod(i, w))
        i = parent[i]
    cells.reverse()
    rows, cols = np.array(cells).T
    points = grid.cell_to_world(rows, cols)
    points[0], points[-1] = start, goal
    return points, {"expansions": expansions, "cost": float(g[goal_i])}


def _line_cells(grid: OccupancyGrid, a, b):
    n = max(2, int(math.ceil(np.linalg.norm(np.asarray(b)-np.asarray(a))/(grid.resolution*0.5)))+1)
    pts = np.linspace(a, b, n)
    return grid.world_to_cell(pts)


def smooth_path(costmap: CostMap, points: np.ndarray, clearance: float = 0.6) -> np.ndarray:
    """Greedy line-of-sight shortcutting that never reduces clearance below
    min(clearance, the clearance the A* path already had on that stretch)."""
    if len(points) <= 2:
        return np.asarray(points, dtype=float)
    rows, cols = costmap.grid.world_to_cell(points)
    along = costmap.distance[rows, cols]
    out = [points[0]]
    i = 0
    while i < len(points)-1:
        j = len(points)-1
        while j > i+1:
            need = min(clearance, float(along[i:j+1].min()))
            r, c = _line_cells(costmap.grid, points[i], points[j])
            if costmap.grid.inside(r, c).all() and costmap.distance[r, c].min() >= need:
                break
            j -= 1
        out.append(points[j])
        i = j
    return np.asarray(out, dtype=float)


def densify(points: np.ndarray, spacing: float = 0.1) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(b-a)/spacing)))
        out.extend(a+(b-a)*k/n for k in range(1, n+1))
    return np.asarray(out)


def plan_path(costmap: CostMap, start_xy, goal_xy, spacing: float = 0.1):
    raw, info = astar(costmap, start_xy, goal_xy)
    smooth = smooth_path(costmap, raw)
    info.update(raw_points=len(raw), vertices=len(smooth),
                length_m=float(np.linalg.norm(np.diff(smooth, axis=0), axis=1).sum()) if len(smooth) > 1 else 0.0)
    return densify(smooth, spacing), info


def path_blocked(costmap: CostMap, path: np.ndarray, from_index: int = 0) -> bool:
    if path is None or len(path) == 0:
        return False
    rows, cols = costmap.grid.world_to_cell(np.asarray(path)[from_index:])
    ok = costmap.grid.inside(rows, cols)
    return bool((~ok).any() or (costmap.distance[rows[ok], cols[ok]] < costmap.lethal_radius*0.9).any())


@dataclass
class PathTracker:
    """Pure pursuit + lateral (vy) cross-track correction + final heading alignment.

    Output is the G1 policy command [vx, vy, yaw_rate] in the body frame.
    """

    max_speed: float = 0.40
    max_lateral: float = 0.15
    max_yaw_rate: float = 0.60
    lookahead: float = 0.60
    lateral_gain: float = 0.8
    goal_tolerance: float = 0.15
    yaw_tolerance: float = 0.10
    rotate_in_place: float = 0.70   # rad of heading error above which vx = 0
    hold_tolerance: float = 0.12    # after reaching, re-centre when pushed/drifted further
    path: np.ndarray = None
    final_yaw: float | None = None
    progress: int = 0
    state: str = "idle"

    def set_path(self, points, final_yaw=None):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0 or not np.isfinite(points).all():
            raise ValueError("path must be a finite (N, 2) array")
        self.path, self.final_yaw, self.progress = points, final_yaw, 0
        self.state = "following"

    def clear(self):
        self.path, self.final_yaw, self.progress, self.state = None, None, 0, "idle"

    def _project(self, xy):
        """Nearest point on the path at or after the current progress segment."""
        pts = self.path
        if len(pts) == 1:
            return 0, pts[0], 0.0
        lo = max(0, self.progress-2)
        hi = min(len(pts)-1, self.progress+60)  # bounded search keeps progress monotone
        a, b = pts[lo:hi], pts[lo+1:hi+1]
        d = b-a
        t = np.clip(np.sum((xy-a)*d, axis=1)/np.maximum(np.sum(d*d, axis=1), 1e-12), 0, 1)
        proj = a+t[:, None]*d
        k = int(np.argmin(np.linalg.norm(proj-xy, axis=1)))
        seg = lo+k
        p = proj[k]
        tangent = d[k]/max(np.linalg.norm(d[k]), 1e-12)
        offset = xy-p
        signed = float(tangent[0]*offset[1]-tangent[1]*offset[0])  # + = robot left of path
        return seg, p, signed

    def _hold(self, xy, yaw):
        """Station keeping: the walking policy drifts a few cm/s even at a zero command."""
        error = self.path[-1]-xy
        dist = float(np.linalg.norm(error))
        if dist > self.hold_tolerance:
            c, s = math.cos(yaw), math.sin(yaw)
            body = np.array([c*error[0]+s*error[1], -s*error[0]+c*error[1]])
            body *= min(0.12, 0.6*dist)/max(dist, 1e-9)
            return np.array([body[0], np.clip(body[1], -self.max_lateral, self.max_lateral), 0.0], dtype=np.float32)
        if self.final_yaw is not None:
            yaw_error = wrap_angle(self.final_yaw-yaw)
            if abs(yaw_error) > 2*self.yaw_tolerance:
                return np.array([0, 0, np.clip(1.2*yaw_error, -self.max_yaw_rate, self.max_yaw_rate)], dtype=np.float32)
        return np.zeros(3, dtype=np.float32)

    def cross_track(self, xy):
        if self.path is None:
            return 0.0
        return self._project(np.asarray(xy, dtype=float))[2]

    def command(self, xy, yaw):
        zero = np.zeros(3, dtype=np.float32)
        if self.state not in ("following", "aligning") or self.path is None:
            return zero
        xy = np.asarray(xy, dtype=float)
        goal = self.path[-1]
        to_goal = float(np.linalg.norm(goal-xy))
        if self.state == "following" and to_goal <= self.goal_tolerance:
            self.state = "aligning" if self.final_yaw is not None else "reached"
        if self.state == "aligning":
            error = wrap_angle(self.final_yaw-yaw)
            if abs(error) <= self.yaw_tolerance:
                self.state = "reached"
                return zero
            return np.array([0, 0, np.clip(1.2*error, -self.max_yaw_rate, self.max_yaw_rate)], dtype=np.float32)
        if self.state == "reached":
            return self._hold(xy, yaw)
        seg, proj, cte = self._project(xy)
        self.progress = max(self.progress, seg)
        # Lookahead point measured along the path from the projection.
        remaining = self.lookahead
        target = goal
        point = proj
        for k in range(seg+1, len(self.path)):
            step = float(np.linalg.norm(self.path[k]-point))
            if step >= remaining:
                target = point+(self.path[k]-point)*(remaining/max(step, 1e-12))
                break
            remaining -= step
            point = self.path[k]
        delta = target-xy
        heading = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-6 else yaw
        alpha = wrap_angle(heading-yaw)
        if abs(alpha) > self.rotate_in_place:
            return np.array([0, 0, np.clip(1.5*alpha, -self.max_yaw_rate, self.max_yaw_rate)], dtype=np.float32)
        # Slow down for heading error and on final approach; never overshoot the goal.
        speed = self.max_speed*max(0.0, math.cos(alpha))**2
        speed = min(speed, max(0.08, 0.8*to_goal))
        dist_la = max(float(np.linalg.norm(delta)), 1e-3)
        yaw_rate = np.clip(2*speed*math.sin(alpha)/dist_la+0.8*alpha*(speed < 0.1),
                           -self.max_yaw_rate, self.max_yaw_rate)
        lateral = np.clip(-self.lateral_gain*cte, -self.max_lateral, self.max_lateral) if to_goal > 0.3 else 0.0
        return np.array([speed, lateral, yaw_rate], dtype=np.float32)
