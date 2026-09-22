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
from g1_factory.routes import resolve_scenario, validate_route, zone_at
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



@pytest.mark.parametrize("y", [0.0, 0.5])
def test_route_validation_checks_segments_and_robot_clearance(y):
    metadata = {
        "config": {"layout": {"robot_clearance": 0.55}},
        "obstacles": [{"id": "test_rack", "bounds": [-0.2, -0.2, 0.2, 0.2]}],
    }
    # Endpoints are clear. The segment either crosses the rack itself, or passes
    # too close for the robot footprint even though its centre line is clear.
    with pytest.raises(ValueError, match="test_rack"):
        validate_route(metadata, [[-2, y], [2, y]])
    validate_route(metadata, [[-2, 0.8], [2, 0.8]])


@pytest.mark.parametrize("points", [[], [[0, 0]], [[0, 0, 0], [1, 1, 1]], [[0, 0], [math.nan, 1]]])
def test_route_validation_rejects_invalid_point_arrays(points):
    with pytest.raises(ValueError):
        validate_route({"config": {"layout": {}}}, points)


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


OFFICE_ZONES = ("office_workstations", "office_meeting", "office_lounge", "office_control")
SCENARIOS = ("warehouse_tour", "office_tour", "warehouse_to_office", "full_tour")


@pytest.mark.parametrize("zone_id", OFFICE_ZONES)
def test_office_rooms_have_physical_walls_and_robot_width_doorways(factory_scene, zone_id):
    mujoco, model, data, metadata = factory_scene
    zone = metadata["zones"][zone_id]
    goal_x, goal_y = zone["goal"]
    inward = 1.0 if goal_y > 0 else -1.0
    door_y = zone["bounds"][1] if inward > 0 else zone["bounds"][3]

    def cast(x):
        # A horizontal ray at torso height from the shared corridor into a room.
        hit = np.array([-1], dtype=np.int32)
        origin = np.array([x, door_y - inward * 0.6, 1.2])
        direction = np.array([0., inward, 0.])
        distance = mujoco.mj_ray(model, data, origin, direction, None, True, -1, hit)
        return distance, int(hit[0])

    # The physical wall beside the opening must stop the ray, not just look solid.
    distance, geom_id = cast(goal_x - 2.0)
    assert 0.45 < distance < 0.7
    assert model.geom_contype[geom_id] != 0
    assert "front_left" in mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    # A 1.1 m wide torso footprint can cross the door plane without collision.
    for offset in (-0.55, 0., 0.55):
        distance, _ = cast(goal_x + offset)
        assert distance < 0 or distance > 1.2


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scenarios_visit_semantic_zones_in_order_without_spawn_completion(factory_scene, scenario):
    _, _, _, metadata = factory_scene
    route = resolve_scenario(metadata, name=scenario)
    points = [route["spawn"][:2], *route["waypoints"]]
    visited = []
    for point in points:
        zone_id = zone_at(metadata, point)
        if zone_id != "transit" and (not visited or visited[-1] != zone_id):
            visited.append(zone_id)
    assert visited == route["expected_zones"]
    assert len(visited) >= 2
    if scenario == "full_tour":
        assert set(visited) == set(metadata["zones"])
    assert route["length_m"] == pytest.approx(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    np.testing.assert_allclose(route["goal"], metadata["zones"][visited[-1]]["goal"])
    follower = WaypointFollower(route["waypoints"])
    follower.command(route["spawn"][:2], route["spawn"][2])
    assert not follower.done
    # Reach each point in sequence. A later goal must never short-circuit the tour.
    for point in route["waypoints"]:
        follower.command(point, 0.)
    assert follower.done


def test_custom_warehouse_office_roundtrip_preserves_requested_zone_order(factory_scene):
    _, _, _, metadata = factory_scene
    warehouse_id = next(key for key in metadata["zones"] if key not in OFFICE_ZONES)
    sequence = [warehouse_id, "office_control", "office_meeting", warehouse_id]
    route = resolve_scenario(metadata, zone_sequence=sequence)
    assert route["expected_zones"] == sequence
    np.testing.assert_allclose(route["spawn"][:2], route["goal"])
    follower = WaypointFollower(route["waypoints"])
    follower.command(route["spawn"][:2], route["spawn"][2])
    assert not follower.done


@pytest.mark.parametrize("arguments", [
    {"name": "unknown"}, {"lab": "unknown"},
    {"zone_sequence": ["office_meeting"]},
    {"zone_sequence": ["office_meeting", "missing"]},
    {"zone_sequence": ["office_meeting", "office_meeting"]},
    {"name": "warehouse_tour", "lab": "lab_a"},
])
def test_unknown_or_ambiguous_scenarios_are_rejected(factory_scene, arguments):
    with pytest.raises(ValueError):
        resolve_scenario(factory_scene[3], **arguments)


def test_changed_layout_obstruction_is_rejected_before_scenario_execution(factory_scene):
    _, _, _, metadata = factory_scene
    route = resolve_scenario(metadata, name="warehouse_to_office")
    points = [route["spawn"][:2], *route["waypoints"]]
    # Put a new physical footprint halfway along the longest required route leg.
    a, b = max(zip(points, points[1:]), key=lambda pair: np.linalg.norm(np.subtract(*pair)))
    x, y = (np.asarray(a) + b) / 2
    modified = copy.deepcopy(metadata)
    modified["obstacles"].append({"id": "temporary_pallet", "bounds": [x - .5, y - .5, x + .5, y + .5]})
    with pytest.raises(ValueError, match="temporary_pallet"):
        resolve_scenario(modified, name="warehouse_to_office")


def test_factory_config_rejects_shelf_on_zone_approach():
    config = copy.deepcopy(load_config(ROOT / "configs/factory.yaml"))
    zone = next(zone for zone in config["zones"] if zone.get("shelves"))
    zone["shelves"][0]["position"] = list(zone["goal"])
    with pytest.raises(ValueError, match="approach route"):
        validate_config(config)


def test_factory_config_rejects_duplicate_semantic_zone_ids():
    config = copy.deepcopy(load_config(ROOT / "configs/factory.yaml"))
    config["zones"][1]["id"] = config["zones"][0]["id"]
    with pytest.raises(ValueError, match="Duplicate zone id"):
        validate_config(config)


def test_large_connected_warehouse_and_office_preserve_g1_policy_interface(factory_scene):
    mujoco, model, data, metadata = factory_scene
    assert set(metadata["zones"]) == {
        "material_storage", "assembly_line", "inspection", "charging", *OFFICE_ZONES,
    }
    assert sum(zone["kind"] == "warehouse" for zone in metadata["zones"].values()) == 4
    assert sum(zone["kind"] == "office" for zone in metadata["zones"].values()) == 4
    xmin, ymin, xmax, ymax = metadata["config"]["layout"]["warehouse_bounds"]
    assert (xmax - xmin) * (ymax - ymin) >= 1500
    # Scene furniture does not change the pretrained 12-joint walking interface.
    assert model.nu == 12
    assert model.opt.timestep == pytest.approx(0.002)
    for name in metadata["camera_names"].values():
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0
    for zone in metadata["zones"].values():
        assert zone_at(metadata, zone["goal"]) == zone["id"]
        for sensor in zone["sensors"]:
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, sensor["site_name"])
            assert site_id >= 0
            np.testing.assert_allclose(data.site_xpos[site_id], sensor["position"])
    # The bridge and central hallway are public transit, not lab-owned rooms.
    assert zone_at(metadata, [26., 0.]) == "transit"
    assert zone_at(metadata, [40., 0.]) == "transit"


def test_warehouse_office_bridge_is_physically_open_at_robot_height(factory_scene):
    mujoco, model, data, _ = factory_scene
    # Probe the actual MJCF from warehouse through the connecting passage and
    # office entrance. Metadata alone would miss an accidentally closed wall.
    for y in (-.55, 0., .55):
        hit = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, np.array([23., y, 1.2]),
                                 np.array([1., 0., 0.]), None, True, -1, hit)
        assert distance < 0 or distance > 6.


def test_labs_choose_scenarios_without_owning_physical_rooms(factory_scene):
    metadata = factory_scene[3]
    assert len(metadata["config"]["labs"]) == 4
    for lab_id in metadata["config"]["labs"]:
        scenario = resolve_scenario(metadata, lab=lab_id)
        assert scenario["lab"] == lab_id
        assert len(scenario["expected_zones"]) >= 2
        assert set(scenario["expected_zones"]) <= metadata["zones"].keys()
    assert not any(zone_id.startswith("lab_") for zone_id in metadata["zones"])


def test_moving_shelf_changes_world_pose_and_obstacle_but_preserves_robot(factory_scene):
    mujoco, model, data, metadata = factory_scene
    before_qpos = data.qpos.copy()
    before_other = copy.deepcopy(metadata["zones"]["material_storage"]["shelves"]["r02"])
    position = [-20., 6.]
    move_shelf(model, data, metadata, "material_storage", "r01", position)
    shelf = metadata["zones"]["material_storage"]["shelves"]["r01"]
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, shelf["body_name"])
    np.testing.assert_allclose(data.xpos[body_id], [*position, 0])
    assert shelf["position"] == position
    obstacle = next(item for item in metadata["obstacles"] if item["id"] == shelf["body_name"])
    expected_half = np.asarray(shelf["size"][:2]) / 2
    np.testing.assert_allclose(obstacle["bounds"], [*(position - expected_half), *(position + expected_half)])
    assert metadata["zones"]["material_storage"]["shelves"]["r02"] == before_other
    np.testing.assert_array_equal(data.qpos, before_qpos)


@pytest.mark.parametrize("target", ["outside_zone", "occupied"])
def test_invalid_shelf_edit_is_atomic(factory_scene, target):
    _, model, data, metadata = factory_scene
    shelves = metadata["zones"]["material_storage"]["shelves"]
    position = [-1000., 0.] if target == "outside_zone" else shelves["r02"]["position"]
    before_positions = data.mocap_pos.copy()
    before_quaternions = data.mocap_quat.copy()
    before_metadata = copy.deepcopy(metadata)
    with pytest.raises(ValueError):
        move_shelf(model, data, metadata, "material_storage", "r01", position)
    np.testing.assert_array_equal(data.mocap_pos, before_positions)
    np.testing.assert_array_equal(data.mocap_quat, before_quaternions)
    assert metadata == before_metadata


def test_temperature_readings_are_labelled_synthetic_and_repeatable(factory_scene):
    metadata = factory_scene[3]
    readings = temperature_readings(metadata, 0)
    assert readings == temperature_readings(metadata, 0)
    assert len(readings) == len(metadata["sensor_names"]) == 8
    assert all(r["synthetic"] is True and np.isfinite(r["temperature_c"]) for r in readings)
    assert {r["zone_id"] for r in readings} == set(metadata["zones"])
    cfg = metadata["config"]["temperature"]
    later = temperature_readings(metadata, cfg["period_s"] / 4)
    for initial, current in zip(readings, later):
        assert current["temperature_c"] - initial["temperature_c"] == pytest.approx(cfg["temporal_amplitude_c"])
