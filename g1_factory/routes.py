"""Named-zone scenarios through explicit shared aisles, independent of walking.

This deliberately transparent baseline is a fixed corridor graph, not a learned
planner. Reject blocked segments rather than claiming obstacle avoidance.
All public coordinates are world-frame metres.
"""
from __future__ import annotations

import math
import numpy as np


def zone_at(metadata, xy):
    x, y = np.asarray(xy, dtype=float)
    for name, zone in metadata["zones"].items():
        x0, y0, x1, y1 = zone["bounds"]
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return "transit"


def segment_hits_box(a, b, bounds, margin=0.55):
    """Exact slab intersection against an expanded 2-D obstacle AABB."""
    lo, hi = np.asarray(bounds[:2])-margin, np.asarray(bounds[2:])+margin
    start, end = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    delta = end-start
    tmin, tmax = 0.0, 1.0
    for axis in range(2):
        if abs(delta[axis]) < 1e-10:
            if start[axis] < lo[axis] or start[axis] > hi[axis]:
                return False
        else:
            near, far = sorted(((lo[axis]-start[axis])/delta[axis], (hi[axis]-start[axis])/delta[axis]))
            tmin, tmax = max(tmin, near), min(tmax, far)
            if tmin > tmax:
                return False
    return True


def validate_route(metadata, points):
    route = np.asarray(points, dtype=float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) < 2 or not np.isfinite(route).all():
        raise ValueError("Route requires at least two finite [x,y] points")
    margin = metadata["config"]["layout"].get("robot_clearance", 0.55)
    for start, end in zip(route[:-1], route[1:]):
        for obstacle in metadata.get("obstacles", []):
            if segment_hits_box(start, end, obstacle["bounds"], margin):
                raise ValueError(f"Route segment {start.tolist()} -> {end.tolist()} blocked by {obstacle['id']}; adjust layout or route")


def resolve_scenario(metadata, name=None, lab=None, zone_sequence=None):
    """Select a lab preset, named scenario or ordered semantic-zone sequence."""
    if sum(value is not None for value in (name, lab, zone_sequence)) > 1:
        raise ValueError("Choose one of scenario, lab, or zone_sequence")
    config = metadata["config"]
    if lab is not None:
        if lab not in config.get("labs", {}):
            raise ValueError(f"Unknown lab {lab}; choose {list(config.get('labs', {}))}")
        assignment = config["labs"][lab]
        name = assignment["scenario"] if isinstance(assignment, dict) else assignment
    if zone_sequence is None:
        name = name or "warehouse_tour"
        if name not in config["scenarios"]:
            raise ValueError(f"Unknown scenario {name}; choose {list(config['scenarios'])}")
        definition = config["scenarios"][name]
        zone_sequence = definition["zones"] if isinstance(definition, dict) else definition
    else:
        name = "custom_" + "_to_".join(zone_sequence)
    sequence = list(zone_sequence)
    if len(sequence) < 2 or any(a == b for a, b in zip(sequence[:-1], sequence[1:])):
        raise ValueError("Use at least two zones; adjacent zones must differ")
    if any(zone not in metadata["zones"] for zone in sequence):
        raise ValueError(f"Unknown semantic zone; choose {list(metadata['zones'])}")
    goals = [np.asarray(metadata["zones"][zone]["goal"], dtype=float) for zone in sequence]
    points = [goals[0].tolist()]
    for goal in goals[1:]:
        previous = np.asarray(points[-1])
        for point in ([float(previous[0]), 0.0], [float(goal[0]), 0.0], goal.tolist()):
            if np.linalg.norm(np.asarray(point)-points[-1]) > 1e-8:
                points.append(point)
    validate_route(metadata, points)
    delta = np.asarray(points[1])-points[0]
    length = float(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1).sum())
    return {"name": name, "lab": lab, "spawn": [*points[0], math.atan2(delta[1], delta[0])],
            "waypoints": points[1:], "expected_zones": sequence, "length_m": length,
            "goal": goals[-1].tolist(), "routing": "fixed central-aisle graph"}


def build_scenarios(metadata):
    return {name: resolve_scenario(metadata, name=name) for name in metadata["config"]["scenarios"]}
