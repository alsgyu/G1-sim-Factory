import json
import math

import numpy as np

from g1_factory import slam_map as S


def test_gravity_alignment_levels_a_tilted_sensor_and_keeps_heading():
    tilt = math.radians(8)
    up = np.array([math.sin(tilt), 0.0, math.cos(tilt)])*9.81  # sensor pitched: gravity leans into +x
    R = S.gravity_alignment(np.tile(up, (50, 1))+np.random.default_rng(0).normal(0, 0.01, (50, 3)))
    assert np.allclose(R@(up/np.linalg.norm(up)), [0, 0, 1], atol=2e-3)
    x = R@np.array([1.0, 0.0, 0.0])
    assert abs(math.atan2(x[1], x[0])) < 1e-3  # heading unchanged
    assert np.allclose(S.gravity_alignment(np.array([[0, 0, 9.81]])), np.eye(3))


def test_floor_height_is_the_dense_level_below_the_sensor():
    rng = np.random.default_rng(1)
    floor = np.c_[rng.uniform(-8, 8, (3000, 2)), np.full(3000, -1.25)+rng.normal(0, 0.005, 3000)]
    wall = np.c_[np.full(800, 5.0), rng.uniform(-5, 5, 800), rng.uniform(-1.25, 2.0, 800)]
    assert abs(S.floor_height(np.r_[floor, wall]) - -1.25) < 0.02
    assert S.floor_height(wall[:10]) is None


def scan_of_room(rng):
    """Floor 0..6 x 0..4 m and a wall along x = 6 (0-2 m high), map frame (floor at z = 0)."""
    floor = np.c_[rng.uniform(0, 6, (4000, 2))*[1, 4/6], rng.normal(0, 0.01, 4000)]
    wall = np.c_[np.full(1500, 6.05), rng.uniform(0, 4, 1500), rng.uniform(0, 2.0, 1500)]
    low = np.c_[rng.uniform(1, 2, (200, 2)), np.full(200, 0.1)]  # below the obstacle band: ignored
    return np.r_[floor, wall, low]


def cell_value(grid, origin, res, x, y):
    return grid[int((y-origin[1])/res), int((x-origin[0])/res)]


def test_slam_grid_marks_floor_free_walls_occupied_and_crops():
    g = S.SlamGrid()
    rng = np.random.default_rng(2)
    for _ in range(3):
        g.add(scan_of_room(rng))
    out = g.build(margin_m=0.5)
    grid, origin = out["grid"], out["origin"]
    assert cell_value(grid, origin, 0.1, 3.0, 2.0) == S.FREE and cell_value(grid, origin, 0.1, 1.5, 1.5) == S.FREE
    assert cell_value(grid, origin, 0.1, 6.05, 2.0) == S.OCCUPIED
    assert cell_value(grid, origin, 0.1, -0.3, 2.0) == S.UNKNOWN  # margin outside the seen area
    assert -1 < origin[0] < 0 and grid.shape[1] < 80
    cloud = out["cloud"]
    assert cloud.shape[1] == 3 and cloud[:, 0].max() < 6.3 and len(cloud) == len(np.unique(cloud, axis=0))


def test_tilted_scans_are_levelled_on_the_floor_plane():
    """A 1.2 deg gravity error lifts the far floor by 0.4 m; the floor-plane fit takes it out."""
    rng = np.random.default_rng(3)
    room = np.r_[np.c_[rng.uniform(0, 20, (200000, 2)), rng.normal(0, 0.01, 200000)],
                 np.c_[np.full(3000, 20.05), rng.uniform(0, 20, 3000), rng.uniform(0, 2.0, 3000)]]
    t = math.radians(1.2)
    tilt = np.array([[math.cos(t), 0, -math.sin(t)], [0, 1, 0], [math.sin(t), 0, math.cos(t)]])
    g = S.SlamGrid()
    g.add(room@tilt.T)
    out = g.build()
    assert abs(math.degrees(math.acos(out["R"][2, 2]))-1.2) < 0.05
    grid, origin = out["grid"], out["origin"]
    floor = [cell_value(grid, origin, 0.1, x, y) for x in (1, 10, 19) for y in (1, 10, 19)]
    assert all(v == S.FREE for v in floor)
    assert cell_value(grid, origin, 0.1, 20.05, 10.0) == S.OCCUPIED
    assert abs(np.median(out["cloud"][out["cloud"][:, 2] < 0.1, 2])) < 0.03


def test_map_yaml_round_trip_keeps_orientation(tmp_path):
    grid = np.full((30, 50), S.UNKNOWN, np.int8)
    grid[5:10, 5:20] = S.FREE
    grid[20, 40] = S.OCCUPIED  # high y, high x
    S.write_map_yaml(tmp_path, grid, (-2.5, 1.0), 0.1)
    back, origin, res = S.read_map_yaml(tmp_path/"map.yaml")
    assert np.array_equal(back, grid) and origin == (-2.5, 1.0) and res == 0.1
    assert (tmp_path/"map.pgm").read_bytes().startswith(b"P5\n50 30\n255\n")


def test_resample_moves_a_grid_into_another_frame():
    grid = np.full((20, 20), S.FREE, np.int8)
    grid[10, 15] = S.OCCUPIED  # map cell centre (1.55, 1.05) with origin (0, 0)
    out = S.resample(grid, (0.0, 0.0), 0.1, S.se2(10.0, 5.0, math.pi/2), (8.0, 4.0, 11.0, 8.0))
    # rotate (1.55, 1.05) by 90 deg -> (-1.05, 1.55); + (10, 5) -> (8.95, 6.55)
    r, c = int((6.55-4.0)/0.1), int((8.95-8.0)/0.1)
    assert out[r, c] == S.OCCUPIED and (out == S.OCCUPIED).sum() == 1
    assert (out == S.UNKNOWN).any()


def test_compare_grids_tolerates_one_cell_offsets():
    truth = np.full((40, 40), S.FREE, np.int8)
    truth[:, 30:32] = S.OCCUPIED
    slam = truth.copy()
    m = S.compare_grids(slam, truth, 0.1)
    assert m["obstacle_precision"] == 1.0 and m["obstacle_recall"] == 1.0 and m["free_marked_free"] == 1.0
    shifted = np.full_like(truth, S.FREE)
    shifted[:, 29:31] = S.OCCUPIED
    m = S.compare_grids(shifted, truth, 0.1)
    assert m["obstacle_precision"] == 1.0 and m["obstacle_recall"] == 1.0
    assert 0 < m["free_marked_occupied"] < 0.05


def test_semantic_annotations_follow_the_map_frame():
    semantic = {"zones": [{"id": "a", "bounds": [10, 5, 12, 9]}], "objects": [
        {"id": "o", "center": [11, 6, 0.5], "aabb": [10.5, 5.5, 0, 11.5, 6.5, 1]}],
        "places": [{"id": "a", "center": [11, 7]}], "edges": [{"a": "a", "b": "b", "door": [12, 7]}],
        "anchors": [{"id": "x", "xy": [10, 5]}], "occupancy": {"bounds": [0, 0, 1, 1]}}
    T_map_world = np.linalg.inv(S.se2(10.0, 5.0, 0.0))
    out = S.transform_semantic(semantic, T_map_world)
    assert out["zones"][0]["bounds"] == [0.0, 0.0, 2.0, 4.0]
    assert out["objects"][0]["center"] == [1.0, 1.0, 0.5] and out["objects"][0]["aabb"] == [0.5, 0.5, 0, 1.5, 1.5, 1]
    assert out["edges"][0]["door"] == [2.0, 2.0] and out["anchors"][0]["xy"] == [0.0, 0.0]
    assert "occupancy" not in out and json.dumps(out)


def test_trajectory_error():
    truth = np.c_[np.linspace(0, 10, 11), np.zeros(11)]
    slam = truth+np.c_[np.zeros(11), np.linspace(0, 0.2, 11)]
    e = S.trajectory_error(slam, truth)
    assert e["final_m"] == 0.2 and e["path_m"] == 10.0 and e["final_drift_pct"] == 2.0


def test_stamps_never_jump_back_at_whole_seconds():
    import pytest
    M = pytest.importorskip("g1_factory.ros2.messages")
    t = M.stamp(85.9999999999)
    assert (t.sec, t.nanosec) == (86, 0)
    times = np.cumsum(np.full(20000, 0.005))  # 200 Hz accumulated in float, as the simulators do
    secs = [M.to_seconds(M.stamp(x)) for x in times]
    assert all(b > a for a, b in zip(secs, secs[1:]))


def test_floor_behind_an_obstacle_is_not_free_from_the_path():
    """Floor seen under a shelf (through the gap below the bottom pallet) is not reachable at body height."""
    rng = np.random.default_rng(4)
    floor = np.c_[rng.uniform(0, 10, (150000, 2)), rng.normal(0, 0.01, 150000)]
    shelf_front = np.c_[rng.uniform(4, 6, 3000), np.full(3000, 6.0), rng.uniform(0.3, 1.2, 3000)]  # face at y = 6
    g = S.SlamGrid()
    g.add(np.r_[floor, shelf_front])
    path = np.c_[np.linspace(1, 9, 30), np.full(30, 2.0)]
    out = g.build(viewpoints=path)
    grid, origin = out["grid"], out["origin"]
    assert cell_value(grid, origin, 0.1, 5.0, 6.0) == S.OCCUPIED
    assert cell_value(grid, origin, 0.1, 5.0, 6.5) == S.UNKNOWN   # behind the face: floor seen, not reachable
    assert cell_value(grid, origin, 0.1, 5.0, 4.0) == S.FREE
    assert cell_value(grid, origin, 0.1, 8.0, 8.0) == S.FREE      # beside the shelf, in view
    assert (g.build()["grid"] == S.FREE).sum() > (grid == S.FREE).sum()  # without viewpoints: floor only


def test_path_viewpoints_are_spaced_along_the_path():
    from g1_factory.ros2.slam_map_node import path_viewpoints  # noqa: needs rclpy
    xy = np.c_[np.linspace(0, 10, 1001), np.zeros(1001)]
    vp = path_viewpoints(xy, spacing=1.0)
    assert len(vp) == 11 and np.allclose(np.diff(vp[:, 0]), 1.0, atol=0.02)
