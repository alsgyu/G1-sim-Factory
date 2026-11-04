"""Topological route over the place graph of the map package.

Nodes are the places (annotated rooms and corridors, unnamed areas); two places are connected through a doorway
where their free space touches (semantic_map.json "edges", built automatically from the SLAM map and the
annotations). The route is a shortest path over the doorways: from the robot to every doorway of its place, between
two doorways of the same place, and from a doorway of the goal's place to the goal, all by straight-line length.
The metric planner then searches only the free space of the places on this route.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math


@dataclass
class Route:
    places: list                        # place ids from the robot's place to the goal's place
    doors: list                         # doorway xy crossed between consecutive places
    length_m: float                     # straight-line estimate through the doorways
    labels: list = field(default_factory=list)

    def text(self) -> str:
        return " -> ".join(self.labels or self.places)


class PlaceGraph:
    def __init__(self, places: list, edges: list):
        self.places = {p["id"]: p for p in places}
        self.edges = [e for e in edges if e["a"] in self.places and e["b"] in self.places]
        self.doors_of = {pid: [] for pid in self.places}
        for k, e in enumerate(self.edges):
            self.doors_of[e["a"]].append(k)
            self.doors_of[e["b"]].append(k)

    def label(self, pid):
        return self.places[pid]["label"]

    def neighbours(self, pid):
        return sorted({e["b"] if e["a"] == pid else e["a"] for e in self.edges if pid in (e["a"], e["b"])})

    def route(self, start_place: str, start_xy, goal_place: str, goal_xy, blocked_doors=()) -> Route | None:
        """Shortest place sequence from start_xy (in start_place) to goal_xy (in goal_place); None if unconnected."""
        if start_place == goal_place:
            return Route([start_place], [], math.dist(start_xy, goal_xy), [self.label(start_place)])
        dist = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])
        door = [tuple(e["door"]) for e in self.edges]
        best = {}
        heap = []
        for k in self.doors_of[start_place]:
            if k in blocked_doors:
                continue
            d = dist(start_xy, door[k])
            best[(k, start_place)] = (d, None)
            heapq.heappush(heap, (d, k, start_place))  # at door k, having come from start_place
        goal_key, goal_cost = None, math.inf
        while heap:
            cost, k, came = heapq.heappop(heap)
            if cost > best[(k, came)][0] or cost >= goal_cost:
                continue
            e = self.edges[k]
            inside = e["b"] if e["a"] == came else e["a"]  # the place we enter through door k
            if inside == goal_place:
                total = cost+dist(door[k], goal_xy)
                if total < goal_cost:
                    goal_key, goal_cost = (k, came), total
                continue
            for j in self.doors_of[inside]:
                if j == k or j in blocked_doors:
                    continue
                new = cost+dist(door[k], door[j])
                if new < best.get((j, inside), (math.inf,))[0]:
                    best[(j, inside)] = (new, (k, came))
                    heapq.heappush(heap, (new, j, inside))
        if goal_key is None:
            return None
        chain = []
        key = goal_key
        while key is not None:
            chain.append(key)
            key = best[key][1]
        chain.reverse()
        places = [start_place]
        doors = []
        for k, came in chain:
            e = self.edges[k]
            places.append(e["b"] if e["a"] == came else e["a"])
            doors.append(door[k])
        return Route(places, doors, goal_cost, [self.label(p) for p in places])
