"""Regression checks for route semantics, action bounds, and editable scenes.

The pure-Python checks need no G1 policy or GPU. Scene checks additionally use
the official robot XML fetched by ``python scripts/fetch_assets.py``.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import pytest

from g1_factory.navigation import (
    RelativeActionFollower,
    WaypointFollower,
    cross_track_error,
    wrap_angle,
    yaw_from_quat,
)
from g1_factory.navila import NavAction, parse_navila_action
from g1_factory.scene import (
    build_scene,
    load_config,
    move_shelf,
    temperature_readings,
    validate_config,
)


ROOT = Path(__file__).resolve().parents[1]


def test_closed_route_cannot_complete_at_its_start():
    follower = WaypointFollower([[2, 0], [2, 2], [0, 0]])
    command = follower.command([0, 0], 0)
    assert follower.index == 0
    assert not follower.done
    assert command[0] > 0

    # Even being at a later waypoint must not skip the first one.
    follower.command([2, 2], math.pi)
    assert follower.index == 0
    follower.command([2, 0], 0)
    assert follower.index == 1
    follower.command([2, 2], math.pi)
    assert follower.index == 2
    np.testing.assert_array_equal(follower.command([0, 0], 0), [0, 0, 0])
    assert follower.done
    # A completed route remains complete if the robot is displaced afterward.
    np.testing.assert_array_equal(follower.command([8, 8], 1), [0, 0, 0])


def test_waypoint_commands_are_in_the_robot_frame():
    # Facing world +Y, reaching world +Y still requires body-forward velocity.
    follower = WaypointFollower([[0, 2]])
    vx, vy, wz = follower.command([0, 0], math.pi / 2)
    assert 0 < vx <= follower.max_speed
    assert vy == 0
    assert abs(wz) < 1e-6

    # A sharp corner must rotate in place rather than cut diagonally through it.
    vx, vy, wz = WaypointFollower([[0, 2]]).command([0, 0], 0)
    assert vx == vy == 0
    assert 0 < wz <= follower.max_yaw_rate


def test_heading_wrap_uses_short_turn_across_pi_boundary():
    heading = math.radians(-179)
    target = [2 * math.cos(heading), 2 * math.sin(heading)]
    follower = WaypointFollower([target])
    vx, vy, wz = follower.command([0, 0], math.radians(179))
    assert vx > 0 and vy == 0
    assert 0 < wz < 0.1
    assert wrap_angle(math.radians(-358)) == pytest.approx(math.radians(2))
    quat = [math.cos(heading / 2), 0, 0, math.sin(heading / 2)]
    assert yaw_from_quat(quat) == pytest.approx(heading)


@pytest.mark.parametrize("points", [[], [[1, 2, 3]], [[math.nan, 0]], [[0, math.inf]]])
def test_invalid_routes_are_rejected(points):
    with pytest.raises(ValueError):
        WaypointFollower(points)


def test_cross_track_error_uses_segments_not_only_waypoints():
    assert cross_track_error([5, 2], [[0, 0], [10, 0]]) == pytest.approx(2)
    assert cross_track_error([12, 0], [[0, 0], [10, 0]]) == pytest.approx(2)
    # Repeated points cannot cause a division by zero or a NaN metric.
    assert cross_track_error([1, 1], [[0, 0], [0, 0], [2, 0]]) == pytest.approx(1)
    assert cross_track_error([3, 4], [[0, 0]]) == pytest.approx(5)


def test_relative_forward_action_uses_observed_pose_and_initial_heading():
    action = RelativeActionFollower(NavAction("move_forward", 0.5), [3, 4], math.pi / 2)
    assert action.command([3, 4], math.pi / 2)[0] > 0
    # Calls without movement cannot finish a distance-based action.
    for _ in range(100):
        action.command([3, 4], math.pi / 2)
    assert not action.done
    np.testing.assert_array_equal(action.command([3, 4.5], math.pi / 2), [0, 0, 0])
    assert action.done


def test_relative_turn_wrap_and_stop():
    action = RelativeActionFollower(NavAction("turn_left", 20), [0, 0], math.radians(175))
    command = action.command([0, 0], math.radians(175))
    np.testing.assert_array_equal(command[:2], [0, 0])
    assert command[2] > 0
    np.testing.assert_array_equal(action.command([0, 0], math.radians(-165)), [0, 0, 0])
    assert action.done
    stop = RelativeActionFollower(NavAction("stop"), [0, 0], 0)
    np.testing.assert_array_equal(stop.command([10, 20], 3), [0, 0, 0])
    assert stop.done


@pytest.mark.parametrize(
    "text, kind, value",
    [
        ("STOP", "stop", 0),
        (" move forward 25 cm. ", "move_forward", 0.25),
        ("move forward 1 meter", "move_forward", 0.5),
        ("Move forward 100 centimeters", "move_forward", 0.5),
        ("turn left 15 degrees", "turn_left", 15),
        ("turn right 90 deg", "turn_right", 30),
    ],
)
def test_navila_native_actions_are_bounded(text, kind, value):
    action = parse_navila_action(text)
    assert action.kind == kind
    assert action.value == pytest.approx(value)


@pytest.mark.parametrize(
    "text",
    [
        "", None, "move forward -1 m", "move forward 0 cm", "move forward NaN m",
        "move forward inf m", "move forward 1000 m", "turn left 0 degrees",
        "turn right 181 degrees", "turn around", "move backward 25 cm",
        "move forward 25 cm then turn left 15 degrees",
        "I think we should move forward 25 cm", "stop\nmove forward 25 cm",
        '{"kind": "move_forward", "value": 100}',
    ],
)
def test_ambiguous_or_unsafe_navila_actions_fail_closed(text):
    with pytest.raises(ValueError):
        parse_navila_action(text)


def test_factory_config_rejects_a_route_through_a_shelf():
    config = copy.deepcopy(load_config(ROOT / "configs/factory.yaml"))
    bay = config["bays"][0]
    # Both endpoints are outside the shelf. The segment itself crosses its body.
    bay["spawn"] = [-2.5, -2.0, 0]
    bay["waypoints"] = [[-2.5, 3.3]]
    with pytest.raises(ValueError, match="shelf"):
        validate_config(config)


def test_factory_config_rejects_two_labs_in_one_cell():
    config = copy.deepcopy(load_config(ROOT / "configs/factory.yaml"))
    config["bays"][1]["grid"] = list(config["bays"][0]["grid"])
    with pytest.raises(ValueError, match="grid cell"):
        validate_config(config)


@pytest.fixture
def factory_scene(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    upstream = ROOT / "external/unitree_rl_gym"
    if not (upstream / "resources/robots/g1_description/g1_12dof.xml").is_file():
        pytest.skip("Official G1 assets absent: run python scripts/fetch_assets.py")
    metadata = build_scene(ROOT / "configs/factory.yaml", upstream, tmp_path / "scene.xml")
    model = mujoco.MjModel.from_xml_path(metadata["scene_path"])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return mujoco, model, data, metadata


def test_factory_scene_compiles_with_four_independent_lab_areas(factory_scene):
    mujoco, model, data, metadata = factory_scene
    assert len(metadata["bays"]) == 4
    assert len({tuple(bay["origin"]) for bay in metadata["bays"].values()}) == 4
    assert model.nu == 12  # Shelves must not add actuators to the walking policy.
    assert model.opt.timestep == pytest.approx(0.002)
    for name in metadata["camera_names"].values():
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0
    for configured in metadata["config"]["bays"]:
        bay = metadata["bays"][configured["id"]]
        np.testing.assert_allclose(np.asarray(bay["waypoints"]) - bay["origin"], configured["waypoints"])
        for sensor in bay["sensors"]:
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, sensor["site_name"])
            assert site_id >= 0
            np.testing.assert_allclose(data.site_xpos[site_id], sensor["position"])


def test_moving_shelf_changes_only_selected_lab_and_preserves_robot(factory_scene):
    mujoco, model, data, metadata = factory_scene
    before_qpos = data.qpos.copy()
    before_other = copy.deepcopy(metadata["bays"]["lab_b"]["shelves"]["west"])
    move_shelf(model, data, metadata, "lab_a", "west", [-2.3, 2.2], math.pi / 2)
    bay = metadata["bays"]["lab_a"]
    shelf = bay["shelves"]["west"]
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, shelf["body_name"])
    np.testing.assert_allclose(data.xpos[body_id], [bay["origin"][0] - 2.3, bay["origin"][1] + 2.2, 0])
    assert shelf["yaw"] == pytest.approx(math.pi / 2)
    assert metadata["bays"]["lab_b"]["shelves"]["west"] == before_other
    np.testing.assert_array_equal(data.qpos, before_qpos)


@pytest.mark.parametrize("position", [[3.8, 1.8], [2.5, 1.8]])
def test_invalid_shelf_edit_is_atomic(factory_scene, position):
    _, model, data, metadata = factory_scene
    before_positions = data.mocap_pos.copy()
    before_quaternions = data.mocap_quat.copy()
    before_metadata = copy.deepcopy(metadata)
    with pytest.raises(ValueError):
        move_shelf(model, data, metadata, "lab_a", "west", position)
    np.testing.assert_array_equal(data.mocap_pos, before_positions)
    np.testing.assert_array_equal(data.mocap_quat, before_quaternions)
    assert metadata == before_metadata


def test_temperature_readings_are_labelled_synthetic_and_repeatable(factory_scene):
    _, _, _, metadata = factory_scene
    readings = temperature_readings(metadata, 0)
    assert readings == temperature_readings(metadata, 0)
    assert len(readings) == len(metadata["sensor_names"]) == 4
    assert all(r["synthetic"] is True and np.isfinite(r["temperature_c"]) for r in readings)
    assert {r["bay_id"] for r in readings} == set(metadata["bays"])
    cfg = metadata["config"]["temperature"]
    later = temperature_readings(metadata, cfg["period_s"] / 4)
    for initial, current in zip(readings, later):
        assert current["temperature_c"] - initial["temperature_c"] == pytest.approx(cfg["temporal_amplitude_c"])
