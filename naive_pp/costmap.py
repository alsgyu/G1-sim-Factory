"""Clearance map of the robot's SLAM grid, with a short-lived layer for obstacles the LiDAR sees but the map lacks.

A cell is traversable when the G1's centre can stand there: at least robot_radius from every cell that is not free
in the SLAM map (occupied, or never seen: unknown floor may hide anything). Paths pay extra inside `clearance`, so they
keep to the middle of aisles and doorways when there is room and still pass through a door 1.6 m wide.
"""
from __future__ import annotations

import numpy as np

from naive_pp import SIM  # noqa: F401  (puts g1_factory on the path)
from g1_factory.localization import distance_field

from naive_pp.package import FREE


class Costmap:
    def __init__(self, grid, origin, resolution, robot_radius=0.40, clearance=1.0, proximity_weight=3.0):
        self.grid = grid
        self.origin = (float(origin[0]), float(origin[1]))
        self.res = float(resolution)
        self.robot_radius = float(robot_radius)
        self.clearance = max(float(clearance), self.robot_radius+self.res)
        self.weight = float(proximity_weight)
        self.max_m = self.clearance+0.2
        self.static = distance_field(grid != FREE, self.res, self.max_m)
        self.dist = self.static.copy()
        self.hits = np.zeros(grid.shape, np.float32)   # time of the last LiDAR return in a cell the map has free
        self._window = None

    @property
    def shape(self):
        return self.grid.shape

    def world_to_cell(self, xy):
        xy = np.asarray(xy, float)
        return (np.floor((xy[..., 1]-self.origin[1])/self.res).astype(int),
                np.floor((xy[..., 0]-self.origin[0])/self.res).astype(int))

    def cell_to_world(self, row, col):
        return np.stack([self.origin[0]+(np.asarray(col)+0.5)*self.res,
                         self.origin[1]+(np.asarray(row)+0.5)*self.res], axis=-1)

    def inside(self, row, col):
        return (0 <= row) & (row < self.shape[0]) & (0 <= col) & (col < self.shape[1])

    def traversable(self) -> np.ndarray:
        return self.dist >= self.robot_radius

    def penalty(self) -> np.ndarray:
        """Step cost factor: 1 in the open, up to 1 + proximity_weight at robot_radius from an obstacle."""
        span = self.clearance-self.robot_radius
        return 1.0+self.weight*np.clip((self.clearance-self.dist)/span, 0.0, 1.0)**2

    def clearance_at(self, xy) -> float:
        r, c = self.world_to_cell(np.asarray(xy, float))
        return float(self.dist[r, c]) if self.inside(r, c) else 0.0

    def nearest_traversable(self, xy, max_m=1.0, mask=None):
        """Centre of the traversable cell nearest to xy (within max_m and the optional mask), or None."""
        r, c = (int(v) for v in self.world_to_cell(np.asarray(xy, float)))
        n = int(np.ceil(max_m/self.res))
        h, w = self.shape
        r0, r1, c0, c1 = max(0, r-n), min(h, r+n+1), max(0, c-n), min(w, c+n+1)
        if r0 >= r1 or c0 >= c1:
            return None
        ok = self.dist[r0:r1, c0:c1] >= self.robot_radius
        if mask is not None:
            ok &= mask[r0:r1, c0:c1]
        rows, cols = np.nonzero(ok)
        if len(rows) == 0:
            return None
        d2 = (rows+r0-r)**2+(cols+c0-c)**2
        k = int(np.argmin(d2))
        if d2[k]*self.res**2 > max_m**2:
            return None
        return self.cell_to_world(rows[k]+r0, cols[k]+c0)

    # ------------------------------------------------------------------ LiDAR layer
    def mark(self, points_xy, stamp, robot_xy=None, self_radius=0.5):
        """Cells hit by LiDAR points (map frame, already cut to obstacle height) that the SLAM map has free."""
        pts = np.asarray(points_xy, float).reshape(-1, 2)
        if robot_xy is not None and len(pts):
            pts = pts[np.hypot(pts[:, 0]-robot_xy[0], pts[:, 1]-robot_xy[1]) > self_radius]
        if len(pts) == 0:
            return 0
        r, c = self.world_to_cell(pts)
        ok = self.inside(r, c)
        r, c = r[ok], c[ok]
        new = self.static[r, c] > 1.5*self.res  # next to a mapped obstacle it is that obstacle, seen again
        self.hits[r[new], c[new]] = stamp
        return int(new.sum())

    def update(self, now, center_xy, decay_s=15.0, window_m=12.0):
        """Fold recent LiDAR obstacles near the robot into the clearance map; older ones are forgotten."""
        self.hits[(self.hits > 0) & (now-self.hits > decay_s)] = 0
        if self._window is not None:
            r0, r1, c0, c1 = self._window
            self.dist[r0:r1, c0:c1] = self.static[r0:r1, c0:c1]
            self._window = None
        if not (self.hits > 0).any():
            return
        r, c = (int(v) for v in self.world_to_cell(np.asarray(center_xy, float)))
        n = int(window_m/self.res)
        h, w = self.shape
        r0, r1, c0, c1 = max(0, r-n), min(h, r+n), max(0, c-n), min(w, c+n)
        if r0 >= r1 or c0 >= c1:
            return
        local = distance_field(self.hits[r0:r1, c0:c1] > 0, self.res, self.max_m)
        self.dist[r0:r1, c0:c1] = np.minimum(self.static[r0:r1, c0:c1], local)
        self._window = (r0, r1, c0, c1)

    def lidar_cells(self) -> int:
        return int((self.hits > 0).sum())
