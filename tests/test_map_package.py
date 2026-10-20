import json

import numpy as np
import pytest

from g1_factory import map_package as P
from g1_factory import slam_map as S
from g1_factory.semantic_world import place_at, topological_hint


def campus_grid():
    """20 x 10 m at 0.1 m: two rooms (west, east) off a corridor along the south, a wall between the rooms,
    each room with a 1.2 m door to the corridor; a store room in the north-east reachable only from the east room."""
    g = np.full((100, 200), S.FREE, np.int8)
    g[[0, -1], :] = S.OCCUPIED
    g[:, [0, -1]] = S.OCCUPIED
    g[30, :] = S.OCCUPIED            # corridor (y < 3) / rooms (y > 3)
    g[30, 40:52] = S.FREE            # west room door, x 4.0-5.2
    g[30, 140:152] = S.FREE          # east room door, x 14.0-15.2
    g[30:, 100] = S.OCCUPIED         # wall between the rooms at x = 10
    g[70, 150:] = S.OCCUPIED         # store room x > 15, y > 7 in the east room ...
    g[70:, 150] = S.OCCUPIED
    g[70, 170:180] = S.FREE          # ... with a door x 17-18
    return g


ANNOTATIONS = {"zones": [
    {"id": "west", "label": "West room", "aliases": ["서쪽 방"], "kind": "room", "rects": [[0.2, 3.2, 9.8, 9.8]]},
    {"id": "east", "label": "East room", "kind": "room", "rects": [[10.2, 3.2, 19.8, 6.9], [10.2, 6.9, 14.9, 9.8]]},
    {"label": "Corridor", "kind": "corridor", "rects": [[0.2, 0.2, 19.8, 2.9]]},
], "anchors": [{"label": "Corridor middle", "xy": [10.0, 1.5]}]}


def test_topology_follows_the_walls_of_the_slam_map():
    sem = P.build_semantic(campus_grid(), (0.0, 0.0), 0.1, ANNOTATIONS)
    ids = {p["id"]: p for p in sem["places"]}
    assert set(ids) == {"west", "east", "corridor", "area_1"}       # the unannotated store room becomes an area
    pairs = {tuple(sorted((e["a"], e["b"]))): e for e in sem["edges"]}
    assert set(pairs) == {("corridor", "west"), ("corridor", "east"), ("area_1", "east")}  # no west-east edge: wall
    door = pairs[("corridor", "west")]["door"]
    assert 4.0 <= door[0] <= 5.2 and abs(door[1]-3.0) < 0.15 and pairs[("corridor", "west")]["width_m"] == pytest.approx(1.2)
    assert pairs[("area_1", "east")]["door"][0] == pytest.approx(17.5, abs=0.5)
    rooms = {z["id"]: z for z in sem["zones"]}
    assert set(rooms) == {"west", "east"} and rooms["west"]["aliases"] == ["서쪽 방"]
    ex, ey = rooms["west"]["entrance"]  # automatic: just inside the door
    assert 3.5 <= ex <= 5.7 and 3.0 < ey < 4.5
    assert sem["anchors"][0]["place"] == "corridor"
    assert json.dumps(sem, ensure_ascii=False)


def test_routes_and_hints_use_the_package_topology():
    sem = P.build_semantic(campus_grid(), (0.0, 0.0), 0.1, ANNOTATIONS)
    assert place_at(sem, (5.0, 6.0)) == "west" and place_at(sem, (5.0, 1.0)) == "corridor"
    assert place_at(sem, (17.0, 8.5)) == "area_1"
    hint = topological_hint(sem, (5.0, 6.0), 0.0, "east")
    assert hint.startswith("Topological route to East room: West room -> Corridor -> East room")
    assert "next opening (into Corridor)" in hint


def test_annotations_are_validated():
    with pytest.raises(ValueError):
        P.load_annotations({"zones": [{"label": "x", "rects": []}]})
    with pytest.raises(ValueError):
        P.load_annotations({"zones": [{"id": "a", "rects": [[0, 0, 1, 1]]}, {"id": "a", "rects": [[2, 2, 3, 3]]}]})
    ann = P.load_annotations({"zones": [{"label": "Pump room 2", "rects": [[5, 5, 1, 1]]}]})
    assert ann["zones"][0]["id"] == "pump_room_2" and ann["zones"][0]["rects"] == [[1, 1, 5, 5]]


def test_package_loads_in_the_robot_frame_or_the_simulator_frame(tmp_path):
    grid = campus_grid()
    S.write_map_yaml(tmp_path, grid, (0.0, 0.0), 0.1)
    T = S.se2(-12.0, 3.8, 0.0)  # world from map: the start pose
    world = S.resample(grid, (0.0, 0.0), 0.1, T, (-12.0, 3.8, 8.0, 13.8))
    S.write_map_yaml(tmp_path, world, (-12.0, 3.8), 0.1, name="map_world")
    (tmp_path/"evaluation.json").write_text(json.dumps({"world_from_map": {"x": -12.0, "y": 3.8, "yaw_rad": 0.0}}))
    (tmp_path/"semantic_map.json").write_text(json.dumps(P.build_semantic(grid, (0.0, 0.0), 0.1, ANNOTATIONS)))
    _, origin, _, sem_map = P.load_package(tmp_path, "map")
    _, origin_w, _, sem_world = P.load_package(tmp_path, "world")
    assert origin == (0.0, 0.0) and origin_w == (-12.0, 3.8)
    regions, labels, entrances, anchors, aliases = P.vln_inputs(sem_world)
    assert regions["west"] == pytest.approx((-11.8, 7.0, -2.2, 13.6))
    assert anchors["Corridor middle"] == pytest.approx((-2.0, 5.3)) and labels["aisle"] == "aisle / corridor"
    assert sem_world["aisle_segments"]["corridor"][0] == pytest.approx(sem_map["aisle_segments"]["corridor"][0]-12.0)


def test_comparison_with_the_simulator_semantic_map():
    sem = P.build_semantic(campus_grid(), (0.0, 0.0), 0.1, ANNOTATIONS)
    truth = {"zones": [{"id": "west", "bounds": [0, 3, 10, 10]}, {"id": "east", "bounds": [10, 3, 20, 7]}],
             "edges": [{"a": "west", "b": "hall", "door": [4.6, 3.0]}, {"a": "east", "b": "hall", "door": [14.6, 3.0]}],
             "anchors": [{"label": "Corridor middle", "xy": [10.0, 1.0]}]}
    check = P.compare_with_truth(sem, truth)
    assert check["room_iou"]["west"] > 0.9 and check["rooms_with_doorway"] == "2/2"
    assert check["doorway_error_m"]["west"] < 0.5 and check["anchor_error_m"]["Corridor middle"] == 0.5
