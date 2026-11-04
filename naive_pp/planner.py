"""Target on the map -> collision-free path for the G1, in the map package's frame.

1. Goal pose. A room or corridor: the spot of its free space nearest the middle that keeps goal_clearance from
   everything (the robot stands inside the room with space around it, not in a doorway or against a machine).
   An anchor: the traversable cell nearest to it. A pose: as given, moved onto free space if it is not.
2. Topological route: the robot's place -> doorways -> the goal's place (topology.PlaceGraph).
3. A* on the 0.1 m grid (8-connected, no corner cutting, octile heuristic), searching only the free space of the
   places on the route plus corridor_m around them; each step costs its length times a clearance penalty, so paths
   keep to the middle of aisles and doors. When that corridor has no path (a doorway blocked by something the map
   lacks), the whole map is searched.
4. Shortcuts: greedy line of sight that never lowers clearance below min(smooth_clearance, what the A* path had
   there), then points every 0.1 m for the path follower.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
import time

import numpy as np

from naive_pp import SIM  # noqa: F401  (puts g1_factory on the path)
from g1_factory.localization import distance_field

from naive_pp.costmap import Costmap
from naive_pp.grounding import Grounder, Grounding, Target
from naive_pp.package import MapPackage
from naive_pp.topology import PlaceGraph, Route

SQRT2 = math.sqrt(2.0)
NEIGHBOURS = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2))


class PlanningError(RuntimeError):
    pass


@dataclass
class PlannerConfig:
    robot_radius: float = 0.40      # the G1's centre keeps this far from any non-free cell (0.23 m half-width + margin)
    clearance: float = 1.0          # inside this distance to obstacles a step costs more
    proximity_weight: float = 3.0
    goal_clearance: float = 0.8     # a room goal keeps this much space around the robot when the room has it
    corridor_m: float = 1.0         # free space this far outside the route's places stays searchable
    smooth_clearance: float = 0.6
    spacing: float = 0.1
    goal_mode: str = "center"       # center: middle of the room | entrance: just inside its main doorway
    max_expansions: int = 4_000_000


@dataclass
class Plan:
    points: np.ndarray              # (N, 2) map frame, spacing apart
    goal: tuple                     # (x, y, yaw) where the robot ends
    target: Target
    route: Route | None
    search: str                     # route: inside the topological corridor | map: whole map
    length_m: float
    expansions: int
    planning_s: float
    start: tuple
    legs: list = field(default_factory=list)

    @property
    def final_yaw(self):
        return self.goal[2]

    def summary(self) -> dict:
        return {"target": {"kind": self.target.kind, "id": self.target.id, "label": self.target.label},
                "goal": [round(float(v), 3) for v in self.goal],
                "route": self.route.labels if self.route else None,
                "route_places": self.route.places if self.route else [],
                "route_text": self.route.text() if self.route else None,
                "doors": [[round(float(v), 2) for v in d] for d in self.route.doors] if self.route else [],
                "search": self.search, "length_m": round(self.length_m, 2), "poses": len(self.points),
                "expansions": self.expansions, "planning_s": round(self.planning_s, 3)}


def astar(costmap: Costmap, passable: np.ndarray, start_rc, goal_rc, max_expansions=4_000_000):
    """Cells of the cheapest 8-connected path; step cost = length * mean penalty of the two cells."""
    h, w = passable.shape
    sr, sc = start_rc
    gr, gc = goal_rc
    if not (passable[sr, sc] and passable[gr, gc]):
        raise PlanningError("start or goal cell is not traversable")
    pas = passable.ravel().tolist()
    pen = costmap.penalty().ravel().tolist()
    n = h*w
    g = [math.inf]*n
    parent = [-1]*n
    closed = bytearray(n)
    start, goal = sr*w+sc, gr*w+gc
    g[start] = 0.0
    heap = [(0.0, 0.0, start)]
    expansions = 0
    push, pop, hypot_k = heapq.heappush, heapq.heappop, SQRT2-2.0
    while heap:
        _, cost, i = pop(heap)
        if closed[i]:
            continue
        if i == goal:
            break
        closed[i] = 1
        expansions += 1
        if expansions > max_expansions:
            raise PlanningError("A* expansion limit reached")
        r, c = divmod(i, w)
        pi = pen[i]
        for dr, dc, step in NEIGHBOURS:
            nr, nc = r+dr, c+dc
            if nr < 0 or nc < 0 or nr >= h or nc >= w:
                continue
            j = nr*w+nc
            if not pas[j] or closed[j]:
                continue
            if dr and dc and not (pas[r*w+nc] and pas[nr*w+c]):
                continue  # no corner cutting past an obstacle
            new = cost+step*0.5*(pi+pen[j])
            if new < g[j]:
                g[j] = new
                parent[j] = i
                ar, ac = abs(nr-gr), abs(nc-gc)
                push(heap, (new+ar+ac+hypot_k*min(ar, ac), new, j))
    else:
        raise PlanningError("no collision-free path")
    cells = []
    i = goal
    while i != -1:
        cells.append(divmod(i, w))
        i = parent[i]
    cells.reverse()
    return np.array(cells), expansions


def _line_cells(costmap: Costmap, a, b):
    n = max(2, int(math.ceil(np.linalg.norm(np.asarray(b)-np.asarray(a))/(costmap.res*0.5)))+1)
    return costmap.world_to_cell(np.linspace(a, b, n))


def smooth(costmap: Costmap, points: np.ndarray, clearance: float) -> np.ndarray:
    """Line-of-sight shortcuts that keep min(clearance, the A* path's own clearance on that stretch)."""
    if len(points) <= 2:
        return np.asarray(points, float)
    rows, cols = costmap.world_to_cell(points)
    along = costmap.dist[rows, cols]
    out = [points[0]]
    i = 0
    while i < len(points)-1:
        j = len(points)-1
        while j > i+1:
            need = max(costmap.robot_radius, min(clearance, float(along[i:j+1].min())))
            r, c = _line_cells(costmap, points[i], points[j])
            if costmap.inside(r, c).all() and costmap.dist[r, c].min() >= need:
                break
            j -= 1
        out.append(points[j])
        i = j
    return np.asarray(out, float)


def densify(points: np.ndarray, spacing: float) -> np.ndarray:
    points = np.asarray(points, float)
    if len(points) < 2:
        return points
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b-a)/spacing)))
        out.extend(a+(b-a)*t/k for t in range(1, k+1))
    return np.asarray(out)


def path_length(points) -> float:
    points = np.asarray(points, float)
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points) > 1 else 0.0


def arrival_heading(points, back_m=1.0) -> float | None:
    """Heading of the last back_m of a path (the direction the robot walks in on arrival)."""
    points = np.asarray(points, float)
    if len(points) < 2:
        return None
    end = points[-1]
    d = np.linalg.norm(points-end, axis=1)
    far = np.nonzero(d >= back_m)[0]
    ref = points[far[-1]] if len(far) else points[0]
    if np.linalg.norm(end-ref) < 1e-6:
        return None
    return math.atan2(end[1]-ref[1], end[0]-ref[0])


class NaivePlanner:
    """Grounding, goal selection, topological route and A* on one map package (no ROS)."""

    def __init__(self, package: MapPackage, config: PlannerConfig | None = None, keywords: dict | None = None):
        self.pkg = package
        self.cfg = config or PlannerConfig()
        self.costmap = Costmap(package.grid, package.origin, package.resolution, self.cfg.robot_radius,
                               self.cfg.clearance, self.cfg.proximity_weight)
        self.graph = PlaceGraph(package.places, package.semantic.get("edges", []))
        self.grounder = Grounder(package.semantic, keywords)
        self.rooms = {z["id"]: z for z in package.semantic.get("zones", [])}

    def ground(self, payload: str) -> Grounding:
        return self.grounder.ground(payload)

    # ------------------------------------------------------------------ goals
    def place_goal(self, pid: str, mode: str | None = None):
        """(x, y) where the robot stands in a room or corridor."""
        mode = mode or self.cfg.goal_mode
        k = self.pkg.place_index(pid)
        cells = self.pkg.labels == k
        trav = cells & self.costmap.traversable()
        if not trav.any():
            raise PlanningError(f"{self.graph.label(pid)} has no free space the robot fits in")
        if mode == "entrance" and self.rooms.get(pid, {}).get("entrance"):
            xy = self.costmap.nearest_traversable(self.rooms[pid]["entrance"], 1.5, cells)
            if xy is not None:
                return tuple(float(v) for v in xy)
        rows, cols = np.nonzero(cells)
        middle = self.costmap.cell_to_world(rows, cols).mean(axis=0)
        dist = np.where(trav, self.costmap.dist, 0.0)
        need = min(self.cfg.goal_clearance, float(dist.max())*0.9)
        rows, cols = np.nonzero(trav & (dist >= need))
        xy = self.costmap.cell_to_world(rows, cols)
        return tuple(float(v) for v in xy[int(np.argmin(np.linalg.norm(xy-middle, axis=1)))])

    def goal_of(self, target: Target):
        """(x, y, yaw or None, place id) of a grounded target."""
        if target.kind == "place":
            x, y = self.place_goal(target.id)
            return x, y, None, target.id
        xy = self.costmap.nearest_traversable(target.xy, 1.5)
        if xy is None:
            raise PlanningError(f"{target.label} at {target.xy} is not on free space the robot fits in")
        return float(xy[0]), float(xy[1]), target.yaw, self.pkg.place_at(xy)

    # ------------------------------------------------------------------ planning
    def corridor(self, route: Route) -> np.ndarray:
        """Free space of the route's places, grown by corridor_m."""
        idx = [self.pkg.place_index(p) for p in route.places]
        inside = np.isin(self.pkg.labels, idx)
        near = distance_field(inside, self.pkg.resolution, self.cfg.corridor_m+self.pkg.resolution)
        return near <= self.cfg.corridor_m

    def leg(self, start_xy, goal_xy, goal_place):
        """(points, route, search, expansions) from start_xy to goal_xy."""
        cm = self.costmap
        trav = cm.traversable()
        start = cm.nearest_traversable(start_xy, 1.0)
        if start is None:
            raise PlanningError(f"the robot at ({start_xy[0]:.2f}, {start_xy[1]:.2f}) is not on free space of the map "
                                "(not localized, or inside an obstacle)")
        start_place = self.pkg.place_at(start)
        route = None
        if start_place and goal_place:
            route = self.graph.route(start_place, tuple(start), goal_place, tuple(goal_xy))
        s_rc = tuple(int(v) for v in cm.world_to_cell(start))
        g_rc = tuple(int(v) for v in cm.world_to_cell(np.asarray(goal_xy, float)))
        attempts = ([("route", trav & self.corridor(route))] if route is not None else [])+[("map", trav)]
        last = None
        for search, passable in attempts:
            try:
                cells, expansions = astar(cm, passable, s_rc, g_rc, self.cfg.max_expansions)
            except PlanningError as exc:
                last = exc
                continue
            raw = cm.cell_to_world(cells[:, 0], cells[:, 1])
            raw[0], raw[-1] = start, goal_xy
            return smooth(cm, raw, self.cfg.smooth_clearance), route, search, expansions
        raise PlanningError(f"no collision-free path to the goal ({last})")

    def plan(self, start_pose, target: Target, waypoints=()) -> Plan:
        """Path from start_pose (x, y, yaw) in the map frame to the target, through the waypoints."""
        began = time.monotonic()
        x, y, yaw, goal_place = self.goal_of(target)
        start_xy = (float(start_pose[0]), float(start_pose[1]))
        stops = []
        for wp in waypoints:
            if wp.kind == "place":  # a place already on the way needs no detour
                here = self.pkg.place_at(start_xy if not stops else stops[-1][:2])
                route = self.graph.route(here, start_xy if not stops else stops[-1][:2], goal_place, (x, y)) \
                    if here and goal_place else None
                if route is not None and wp.id in route.places:
                    continue
            wx, wy, _, wplace = self.goal_of(wp)
            stops.append((wx, wy, wplace))
        stops.append((x, y, goal_place))
        pieces, routes, legs, expansions, searches = [], [], [], 0, set()
        here = start_xy
        for sx, sy, splace in stops:
            pts, route, search, n = self.leg(here, (sx, sy), splace)
            pieces.append(pts if not pieces else pts[1:])
            legs.append({"goal": [round(sx, 2), round(sy, 2)], "search": search, "route": route.labels if route else None})
            if route is not None:
                routes.append(route)
            expansions += n
            searches.add(search)
            here = (sx, sy)
        vertices = np.concatenate(pieces)
        points = densify(vertices, self.cfg.spacing)
        if yaw is None:
            yaw = arrival_heading(points)
            if yaw is None:
                yaw = float(start_pose[2])
        merged = None
        if routes:
            places, doors, labels = list(routes[0].places), list(routes[0].doors), list(routes[0].labels)
            for r in routes[1:]:
                skip = 1 if places and r.places and places[-1] == r.places[0] else 0
                places += r.places[skip:]
                labels += r.labels[skip:]
                doors += r.doors
            merged = Route(places, doors, sum(r.length_m for r in routes), labels)
        return Plan(points, (x, y, float(yaw)), target, merged, "map" if "map" in searches else "route",
                    path_length(points), expansions, time.monotonic()-began, tuple(start_pose), legs)

    def blocked(self, points, from_index=0, ahead_m=8.0) -> bool:
        """True when a stretch of the path ahead now passes closer than robot_radius to an obstacle."""
        pts = np.asarray(points)[from_index:]
        if len(pts) == 0:
            return False
        pts = pts[:max(1, int(ahead_m/self.cfg.spacing))]
        r, c = self.costmap.world_to_cell(pts)
        ok = self.costmap.inside(r, c)
        return bool((~ok).any() or (self.costmap.dist[r[ok], c[ok]] < self.cfg.robot_radius*0.9).any())
