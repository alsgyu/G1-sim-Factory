import json
import math
import time

import numpy as np
import pytest
import yaml

from conftest import HERE, two_rooms
from naive_pp.grounding import Target
from naive_pp.package import FREE
from naive_pp.planner import NaivePlanner, PlanningError
from naive_pp.topology import PlaceGraph

TASKS = yaml.safe_load((HERE/"configs/instructions.yaml").read_text(encoding="utf-8"))["tasks"]


def clearance_along(pp, points):
    r, c = pp.costmap.world_to_cell(np.asarray(points))
    return pp.costmap.static[r, c]


def place(pid):
    return Target("place", pid, pid, pid)


def test_two_rooms_path_goes_through_the_door_and_ends_inside(tmp_path):
    pp = NaivePlanner(two_rooms(tmp_path))
    plan = pp.plan((3.0, 2.0, 0.0), place("room_b"))
    assert plan.route.places == ["room_a", "room_b"] and plan.search == "route"
    crossing = plan.points[np.argmin(np.abs(plan.points[:, 0]-9.1))]
    assert 4.0+0.4 <= crossing[1] <= 5.8-0.4  # through the gap, the centre at least robot_radius from both jambs
    assert clearance_along(pp, plan.points).min() >= pp.cfg.robot_radius-1e-6
    assert pp.pkg.place_at(plan.goal[:2]) == "room_b"
    assert pp.costmap.clearance_at(plan.goal[:2]) >= pp.cfg.goal_clearance-1e-6
    assert np.max(np.linalg.norm(np.diff(plan.points, axis=0), axis=1)) <= pp.cfg.spacing+1e-6
    assert abs(math.atan2(math.sin(plan.goal[2]), math.cos(plan.goal[2]))) < math.pi/2  # arrives facing into B


def test_a_door_too_narrow_for_the_robot_is_not_used(tmp_path):
    pp = NaivePlanner(two_rooms(tmp_path, doors=((4.0, 4.6), (7.0, 8.8))))
    plan = pp.plan((3.0, 2.0, 0.0), place("room_b"))
    crossing = plan.points[np.argmin(np.abs(plan.points[:, 0]-9.1))]
    assert 7.0 < crossing[1] < 8.8


def test_lidar_obstacle_in_the_door_reroutes_and_then_fails(tmp_path):
    pp = NaivePlanner(two_rooms(tmp_path, doors=((2.0, 3.8), (6.0, 7.8))))
    first = pp.plan((3.0, 5.0, 0.0), place("room_b"))
    used = 2.9 if first.points[np.argmin(np.abs(first.points[:, 0]-9.1)), 1] < 5 else 6.9
    pp.costmap.mark(np.c_[np.full(20, 9.1), np.linspace(used-0.9, used+0.9, 20)], stamp=1.0)
    pp.costmap.update(1.0, (9.0, 5.0))
    assert pp.blocked(first.points)
    second = pp.plan((3.0, 5.0, 0.0), place("room_b"))
    crossing = second.points[np.argmin(np.abs(second.points[:, 0]-9.1)), 1]
    assert abs(crossing-used) > 2.0 and not pp.blocked(second.points)
    other = 6.9 if used < 5 else 2.9
    pp.costmap.mark(np.c_[np.full(20, 9.1), np.linspace(other-0.9, other+0.9, 20)], stamp=2.0)
    pp.costmap.update(2.0, (9.0, 5.0))
    with pytest.raises(PlanningError):
        pp.plan((3.0, 5.0, 0.0), place("room_b"))
    pp.costmap.update(30.0, (9.0, 5.0))  # LiDAR obstacles are forgotten after decay_s
    pp.plan((3.0, 5.0, 0.0), place("room_b"))


def test_start_off_free_space_is_snapped_or_refused(tmp_path):
    pp = NaivePlanner(two_rooms(tmp_path))
    plan = pp.plan((0.45, 5.0, 0.0), place("room_b"))  # 0.25 m from the wall: inside the robot radius
    assert np.hypot(*(plan.points[0]-[0.45, 5.0])) < 0.5
    with pytest.raises(PlanningError):
        pp.plan((-5.0, 5.0, 0.0), place("room_b"))


def test_anchor_and_pose_targets(tmp_path):
    pp = NaivePlanner(two_rooms(tmp_path))
    g = pp.ground("go to the B corner")
    plan = pp.plan((3.0, 2.0, 0.0), g.target)
    assert np.hypot(plan.goal[0]-18.5, plan.goal[1]-1.5) < 0.2
    plan = pp.plan((3.0, 2.0, 0.0), Target("pose", "pose", "pose", None, (15.0, 8.0), 1.2))
    assert plan.goal == (15.05, 8.05, 1.2) or np.allclose(plan.goal, (15.0, 8.0, 1.2), atol=0.1)


def test_route_through_the_place_graph():
    places = [{"id": k, "label": k.upper()} for k in "abcde"]
    edges = [{"a": "a", "b": "b", "door": [1, 0]}, {"a": "b", "b": "c", "door": [2, 0]},
             {"a": "c", "b": "d", "door": [3, 0]}, {"a": "b", "b": "d", "door": [2, 5]}]
    g = PlaceGraph(places, edges)
    r = g.route("a", (0, 0), "d", (4, 0))
    assert r.places == ["a", "b", "c", "d"] and r.doors == [(1, 0), (2, 0), (3, 0)]
    assert r.text() == "A -> B -> C -> D"
    assert g.route("a", (0, 0), "d", (4, 0), blocked_doors={1}).places == ["a", "b", "d"]
    assert g.route("a", (0, 0), "e", (9, 9)) is None
    assert g.route("c", (0, 0), "c", (3, 4)).length_m == 5.0


@pytest.mark.parametrize("name", sorted(TASKS))
def test_every_listed_instruction_plans_a_safe_path_into_the_place(campus, name):
    task = TASKS[name]
    text = task["instruction"] if isinstance(task["instruction"], str) else json.dumps(task["instruction"])
    g = campus.ground(text)
    began = time.monotonic()
    plan = campus.plan(tuple(task["start"]), g.target, g.waypoints)
    assert time.monotonic()-began < 3.0
    assert campus.pkg.place_at(plan.goal[:2]) == task["expect"]
    assert clearance_along(campus, plan.points).min() >= campus.cfg.robot_radius-1e-6
    r, c = campus.costmap.world_to_cell(plan.points)
    assert (campus.pkg.grid[r, c] == FREE).all()
    assert np.hypot(*(plan.points[0]-task["start"][:2])) < 1.0
    assert np.allclose(plan.points[-1], plan.goal[:2])
    if g.target.kind == "place":
        assert plan.route.places[-1] == task["expect"]


def test_the_colleagues_route_on_the_campus(campus):
    g = campus.ground("Go to the operations control room and check whether the status indicator lights on the "
                      "server rack are on.")
    plan = campus.plan((0.0, 0.0, 0.0), g.target)
    assert plan.route.places == ["material_storage", "warehouse_aisle", "bridge", "office_corridor", "office_control"]
    assert plan.search == "route" and 55 < plan.length_m < 70
    assert campus.costmap.clearance_at(plan.goal[:2]) >= campus.cfg.goal_clearance-1e-6
