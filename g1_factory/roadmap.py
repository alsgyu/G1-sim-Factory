"""Waypoint graph on the robot's SLAM map, and the candidate waypoints offered to the VLN.

The SLAM occupancy map tells where the floor is walkable; it does not say where
a target is. The VLN high level chooses, decision by decision, which reachable
waypoint to walk to next. This module only enumerates options:

* RoadMap: lattice nodes with clearance, labelled with the annotated region they lie in.
* candidates(): from the current pose, the farthest waypoint reachable by a
  straight, obstacle-free segment in each direction sector (<= max_range m).
  Straight segments mean the path is exactly the sequence of VLN choices.
* project(): where each candidate's floor point appears in the head camera image,
  so it can be drawn as a numbered marker for the VLM ("visual prompting").
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

import numpy as np

from .navigation import wrap_angle
from .planning import CostMap, _line_cells


@dataclass
class Candidate:
    id: int
    xy: np.ndarray
    distance: float
    bearing_deg: float      # + = left of the robot's heading
    region: str
    visited: bool
    pixel: tuple | None = None   # (u, v) in the camera image when in view
    compass: str = ""            # map compass direction of the move (+y = north, +x = east)
    route_gain: float | None = None   # metres closer (walking distance) to the current step's region


def region_at(regions: dict, xy) -> str:
    x, y = float(xy[0]), float(xy[1])
    for name, (x0, y0, x1, y1) in regions.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return "aisle"


@dataclass
class RoadMap:
    costmap: CostMap
    regions: dict                 # name -> [xmin, ymin, xmax, ymax] (annotated zones)
    spacing: float = 1.5
    clearance: float = 0.6
    nodes: np.ndarray = field(init=False)
    labels: list = field(init=False)

    def __post_init__(self):
        grid = self.costmap.grid
        xs = np.arange(grid.origin[0]+self.spacing/2, grid.origin[0]+grid.width*grid.resolution, self.spacing)
        ys = np.arange(grid.origin[1]+self.spacing/2, grid.origin[1]+grid.height*grid.resolution, self.spacing)
        xx, yy = np.meshgrid(xs, ys)
        pts = np.stack([xx.ravel(), yy.ravel()], axis=1)
        rows, cols = grid.world_to_cell(pts)
        ok = grid.inside(rows, cols)
        pts, rows, cols = pts[ok], rows[ok], cols[ok]
        keep = self.costmap.distance[rows, cols] >= self.clearance
        ii, jj = np.meshgrid(np.arange(len(xs)), np.arange(len(ys)))
        lattice = np.stack([jj.ravel(), ii.ravel()], axis=1)[ok][keep]
        self.nodes = pts[keep]
        self.labels = [region_at(self.regions, p) for p in self.nodes]
        self._index = {(int(r), int(c)): k for k, (r, c) in enumerate(lattice)}
        self._lattice = lattice
        self._edges = None
        self._route_cache = {}

    # ------------------------------------------------------------ walking routes
    def edges(self):
        """8-connected lattice edges whose straight segment keeps clearance (built once)."""
        if self._edges is None:
            self._edges = [[] for _ in range(len(self.nodes))]
            for k, (r, c) in enumerate(self._lattice):
                for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
                    j = self._index.get((int(r)+dr, int(c)+dc))
                    if j is not None and self.segment_clear(self.nodes[k], self.nodes[j]):
                        w = float(np.linalg.norm(self.nodes[k]-self.nodes[j]))
                        self._edges[k].append((j, w))
                        self._edges[j].append((k, w))
        return self._edges

    def route_distance(self, region: str) -> np.ndarray:
        """Walking distance from every waypoint to the annotated region (multi-source Dijkstra).

        This uses only the SLAM map and the region annotation; it is a hint shown
        to the VLN, not a path the robot follows.
        """
        if region not in self._route_cache:
            dist = np.full(len(self.nodes), np.inf)
            heap = [(0.0, k) for k, label in enumerate(self.labels) if label == region]
            for _, k in heap:
                dist[k] = 0.0
            heapq.heapify(heap)
            edges = self.edges()
            while heap:
                d, k = heapq.heappop(heap)
                if d > dist[k]:
                    continue
                for j, w in edges[k]:
                    if d+w < dist[j]:
                        dist[j] = d+w
                        heapq.heappush(heap, (d+w, j))
            self._route_cache[region] = dist
        return self._route_cache[region]

    def nearest_node(self, xy) -> int:
        d = np.linalg.norm(self.nodes-np.asarray(xy, float), axis=1)
        for k in np.argsort(d)[:12]:
            if self.segment_clear(xy, self.nodes[k], 0.3):
                return int(k)
        return int(np.argmin(d))

    def segment_clear(self, a, b, clearance=0.45) -> bool:
        rows, cols = _line_cells(self.costmap.grid, np.asarray(a, float), np.asarray(b, float))
        grid = self.costmap.grid
        if not grid.inside(rows, cols).all():
            return False
        return bool(self.costmap.distance[rows, cols].min() >= clearance)

    def candidates(self, xy, yaw, max_range=8.0, min_range=1.5, sectors=12, visited=(), region=None):
        """Farthest straight-line-reachable waypoint per direction sector, numbered
        from the most frontal one."""
        xy = np.asarray(xy, float)
        delta = self.nodes-xy
        dist = np.linalg.norm(delta, axis=1)
        near = np.nonzero((dist >= min_range) & (dist <= max_range))[0]
        best = {}
        for idx in near[np.argsort(-dist[near], kind="stable")]:  # farthest first
            bearing = float(np.degrees(wrap_angle(math.atan2(delta[idx, 1], delta[idx, 0])-yaw)))
            sector = int((bearing+180.0)/360.0*sectors) % sectors
            if sector not in best and self.segment_clear(xy, self.nodes[idx]):
                best[sector] = (idx, bearing)
        visited = np.asarray(visited, float).reshape(-1, 2)
        route = self.route_distance(region) if region else None
        here = route[self.nearest_node(xy)] if route is not None else None
        result = []
        for n, (idx, bearing) in enumerate(sorted(best.values(), key=lambda item: abs(item[1])), start=1):
            p = self.nodes[idx]
            seen = bool(len(visited)) and float(np.min(np.linalg.norm(visited-p, axis=1))) < 1.2
            gain = None
            if route is not None and np.isfinite(here) and np.isfinite(route[idx]):
                gain = float(here-route[idx])
            heading = math.atan2(delta[idx, 1], delta[idx, 0])
            result.append(Candidate(n, p.copy(), float(dist[idx]), bearing, self.labels[idx], seen,
                                    compass=compass_name(heading), route_gain=gain))
        return result


def project_points(points_xyz: np.ndarray, map_from_cam: np.ndarray, fx, fy, cx, cy, width, height):
    """Map-frame 3-D points -> pixel (u, v) or None, for an optical camera frame (z forward)."""
    cam_from_map = np.linalg.inv(map_from_cam)
    out = []
    for p in points_xyz:
        q = cam_from_map@np.array([p[0], p[1], p[2], 1.0])
        if q[2] < 0.3:
            out.append(None)
            continue
        u, v = fx*q[0]/q[2]+cx, fy*q[1]/q[2]+cy
        out.append((float(u), float(v)) if 0 <= u < width and 0 <= v < height else None)
    return out


def compass_name(heading: float) -> str:
    """Map heading (rad, 0 = +x = east, counter-clockwise) -> 8-point compass name."""
    names = ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]
    return names[int(round(math.degrees(heading)/45.0)) % 8]


def describe_direction(bearing_deg: float) -> str:
    b = bearing_deg
    if abs(b) <= 20:
        return "ahead"
    if abs(b) >= 160:
        return "behind"
    side = "left" if b > 0 else "right"
    if abs(b) < 70:
        return f"ahead-{side}"
    if abs(b) <= 110:
        return side
    return f"behind-{side}"


def draw_candidates(rgb: np.ndarray, candidates) -> np.ndarray:
    """Numbered markers on the floor points of in-view candidates (visual prompt)."""
    from PIL import Image, ImageDraw, ImageFont
    image = Image.fromarray(rgb).convert("RGB")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
    except OSError:
        font = ImageFont.load_default()
    for c in candidates:
        if c.pixel is None:
            continue
        u, v = c.pixel
        r = 16
        draw.ellipse((u-r, v-r, u+r, v+r), fill=(255, 255, 255), outline=(220, 30, 30), width=3)
        draw.text((u, v), str(c.id), fill=(0, 0, 0), anchor="mm", font=font)
    return np.asarray(image)
