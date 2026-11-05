"""Counterfactual actions for one aligned observation (simulation experiments)."""
from .vln_navigator import approach_pose, behind_partition

MODES = ("depth", "gate", "edge", "legacy", "full")


def alternatives(target, near, robot, cm, standoff):
    exact = behind_partition(target, robot, cm.occupied, grid=cm.grid)
    sampled = behind_partition(target, robot, cm.occupied)
    depth_pose = approach_pose(target, near, robot, standoff)
    edge_pose = approach_pose(target, near, robot, standoff, cm.occupied)
    pairs = [(False, depth_pose), (exact, depth_pose), (False, edge_pose),
             (sampled, edge_pose), (exact, edge_pose)]
    options = {}
    for mode, (hold, goal) in zip(MODES, pairs):
        snap = None if hold else cm.nearest_traversable(goal[:2], 1.5)
        options[mode] = {"hold": bool(hold), "goal": None if hold else list(goal),
                         "snap": None if snap is None else list(map(float, snap))}
    return options
