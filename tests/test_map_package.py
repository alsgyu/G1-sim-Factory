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


def test_export_bundle_gives_an_external_agent_the_map_but_no_simulator_truth(tmp_path):
    import hashlib
    pkg, grid = tmp_path/"campus", campus_grid()
    pkg.mkdir()
    S.write_map_yaml(pkg, grid, (0.0, 0.0), 0.1)
    S.write_map_yaml(pkg, grid, (-12.0, 3.8), 0.1, name="map_world")
    (pkg/"annotations.json").write_text(json.dumps(ANNOTATIONS))
    (pkg/"semantic_map.json").write_text(json.dumps(P.build_semantic(grid, (0.0, 0.0), 0.1, ANNOTATIONS)))
    (pkg/"meta.json").write_text(json.dumps({"name": "campus", "frame": "map: start pose", "resolution_m": 0.1,
                                             "world_from_map": {"x": -12.0, "y": 3.8, "yaw_rad": 0.0},
                                             "trajectory": {"ate_rmse_m": 0.1}, "map": {"obstacle_precision": 0.98}}))
    for truth in ("evaluation.json", "semantic_truth.json", "trajectory_truth.csv", "trajectory.csv", "cloud.pcd"):
        (pkg/truth).write_text("{}")
    m = P.export_bundle(pkg, tmp_path/"out", "abc1234")
    out = tmp_path/"out"
    assert sorted(p.name for p in out.iterdir()) == sorted(
        ["map.yaml", "map.pgm", "semantic_map.json", "annotations.json", "meta.json", "README.md", "manifest.json",
         "SHA256SUMS"])
    assert json.loads((out/"meta.json").read_text()) == {"name": "campus", "frame": "map: start pose",
                                                         "resolution_m": 0.1}
    for line in (out/"SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ")
        assert hashlib.sha256((out/name).read_bytes()).hexdigest() == digest
    readme = (out/"README.md").read_text()
    assert "West room" in readme and "서쪽 방" in readme and m["version"] in readme and "abc1234" in readme
    assert m["counts"] == {"zones": 2, "places": 4, "edges": 3, "anchors": 1} and m["size_cells"] == [200, 100]
    assert P.export_bundle(pkg, tmp_path/"again")["version"] == m["version"]  # same map, same version
    with pytest.raises(FileExistsError):
        P.export_bundle(pkg, out)
    sem = json.loads((pkg/"semantic_map.json").read_text())
    (pkg/"semantic_map.json").write_text(json.dumps({**sem, "objects": [{"id": "tank", "center": [1, 2, 0]}]}))
    with pytest.raises(ValueError):  # the simulator's objects never leave
        P.export_bundle(pkg, tmp_path/"leak")
    (pkg/"semantic_map.json").unlink()
    with pytest.raises(FileNotFoundError, match="build_map_package"):
        P.export_bundle(pkg, tmp_path/"none")


def test_a_package_the_robot_cannot_route_on_is_refused(tmp_path):
    grid, o = campus_grid(), (0.0, 0.0)
    sem = P.build_semantic(grid, o, 0.1, ANNOTATIONS)
    assert P.check_semantic(grid, o, 0.1, sem) == []
    # an Isaac map from before the LiDAR fixes: walls only, the floor never seen. topology.png still looks right.
    walls = np.where(grid == S.OCCUPIED, S.OCCUPIED, S.UNKNOWN).astype(np.int8)
    old = P.build_semantic(walls, o, 0.1, ANNOTATIONS)
    assert old["edges"] == [] and all(p["free_m2"] == 0 and p["center"] is None for p in old["places"])
    problems = P.check_semantic(walls, o, 0.1, old)
    assert len(problems) == 1 and "no free cells" in problems[0]
    # the west door shut: the west room has no route from the others
    shut = grid.copy()
    shut[30, 40:52] = S.OCCUPIED
    problems = P.check_semantic(shut, o, 0.1, P.build_semantic(shut, o, 0.1, ANNOTATIONS))
    assert len(problems) == 1 and "no path from the other places to west:" in problems[0]
    # a room drawn where this map has no floor
    ghost = {**ANNOTATIONS, "zones": ANNOTATIONS["zones"]+[{"id": "ghost", "kind": "room", "rects": [[30, 30, 35, 35]]}]}
    problems = P.check_semantic(grid, o, 0.1, P.build_semantic(grid, o, 0.1, ghost))
    assert problems == ["no free floor under ghost: drawn on another map, or that floor was not seen"]
    # remapped and not rebuilt: the semantic map of the old map does not fit the new one
    assert "built on another map" in P.check_semantic(grid, (1.0, 0.0), 0.1, sem)[0]
    pkg = tmp_path/"campus"
    pkg.mkdir()
    S.write_map_yaml(pkg, walls, o, 0.1)
    (pkg/"semantic_map.json").write_text(json.dumps(old))
    with pytest.raises(ValueError, match="not ready to hand over:\n  - the map has no free cells"):
        P.export_bundle(pkg, tmp_path/"out")
    assert not (tmp_path/"out").exists()
