"""Map-guided VLN high level without a VLM server: candidates, route hints, rules, plate checks."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from g1_factory.planning import CostMap, OccupancyGrid
from g1_factory.roadmap import RoadMap, compass_name, project_points, region_at
from g1_factory.scene import build_scene, load_config
from g1_factory.sensors import mid360_directions
from g1_factory.spawn import perturbed_spawn
from g1_factory.vlm import VLMCall
from g1_factory.vln_navigator import NavStep, NavTask, VLNNavigator, already_in_front, approach_pose, grounded_label

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def roadmap(tmp_path_factory):
    upstream = ROOT/"external/unitree_rl_gym"
    if not (upstream/"resources/robots/g1_description/g1_12dof.xml").is_file():
        pytest.skip("Official G1 assets absent: run python scripts/fetch_assets.py")
    meta = build_scene(ROOT/"configs/factory.yaml", upstream, tmp_path_factory.mktemp("scene")/"scene.xml")
    regions = {z: v["bounds"] for z, v in meta["zones"].items()}
    return RoadMap(CostMap(OccupancyGrid.from_scene(meta, 0.1)), regions)


class FakeVLM:
    def __init__(self, replies):
        self.replies, self.prompts = list(replies), []

    def chat(self, system, text, images=(), max_tokens=400):
        self.prompts.append(text)
        return VLMCall(self.replies.pop(0), 0.1, len(text), 10, len(images))


def navigator(roadmap, replies, steps, label=""):
    config = load_config(ROOT/"configs/factory.yaml")
    labels = {z["id"]: z["label"] for z in config["zones"]} | {"aisle": "aisle / corridor"}
    entrances = {z["id"]: tuple(z["goal"]) for z in config["zones"]}
    nav = VLNNavigator(FakeVLM(replies), roadmap, labels, entrances)
    nav.start(NavTask("test instruction", steps, "target object", "?", "en", label))
    return nav


def frame():
    cam = np.eye(4)  # optical frame: z forward = map +x, x right = map -y, y down = map -z
    cam[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    cam[:3, 3] = [0, 0, 1.24]
    return np.zeros((480, 640, 3), np.uint8), cam, (500.0, 500.0, 319.5, 239.5, 640, 480)


@pytest.mark.parametrize("label, instruction, expected", [
    ("STATION 2", "두 번째 비전 검사 장비 위에 부품이 있는지 봐줘", "STATION 2"),
    ("CHARGER 3", "충전 구역의 3번 충전 도크를 확인해", "CHARGER 3"),
    ("STATION 2", "Go to the second inspection station", "STATION 2"),
    ("CONVEYOR A", "조립 라인 컨베이어 위에 상자가 있는지 확인해", ""),   # the model guessed "A"
    ("SERVER RACK 1", "서버 랙 표시등을 확인해", ""),
    ("FIRE EXTINGUISHER", "빨간 소화기함까지 가", ""),
])
def test_plate_label_is_kept_only_when_the_instruction_names_it(label, instruction, expected):
    assert grounded_label(label, instruction) == expected


def test_route_hint_prefers_the_way_around_walls(roadmap):
    # East of (22.2, -5.8) is the warehouse wall; the office is reached via the aisle to the north.
    cands = roadmap.candidates((22.2, -5.8), 0.0, region="office_control")
    best = max(cands, key=lambda c: c.route_gain or -1e9)
    assert best.compass in ("north", "north-west", "north-east")
    assert best.route_gain > 0
    assert all(roadmap.segment_clear((22.2, -5.8), c.xy) for c in cands)
    assert compass_name(0.0) == "east" and compass_name(math.pi/2) == "north"


def test_candidates_project_into_the_image(roadmap):
    rgb, cam, intr = frame()
    cam[:3, 3] = [-4.8, 1.8, 1.24]
    cands = roadmap.candidates((-4.8, 1.8), 0.0)
    pixels = project_points(np.array([[c.xy[0], c.xy[1], 0.0] for c in cands]), cam, *intr)
    ahead = [c for c, px in zip(cands, pixels) if px is not None]
    assert ahead and all(abs(c.bearing_deg) < 50 for c in ahead)


def test_move_away_from_the_step_region_is_rejected_and_retried(roadmap):
    rgb, cam, intr = frame()
    cam[:3, 3] = [10.2, -1.2, 1.24]
    cands = roadmap.candidates((10.2, -1.2), 0.0, region="assembly_line")
    away = min(cands, key=lambda c: c.route_gain)
    toward = max(cands, key=lambda c: c.route_gain)
    nav = navigator(roadmap, [f'{{"action": "move", "waypoint": {away.id}}}',
                              f'{{"action": "move", "waypoint": {toward.id}}}'],
                    [NavStep("go to the assembly line", "assembly_line")])
    d = nav.decide(np.array([10.2, -1.2]), 0.0, rgb, cam, intr)
    assert d.action == "move" and d.waypoint.id == toward.id
    assert "away from" in nav.client.prompts[1]


def test_approach_needs_the_named_plate(roadmap):
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"action": "approach", "bbox_2d": [400, 300, 600, 700]}', '{"text": "STATION 3"}',
                              '{"action": "approach", "bbox_2d": [100, 300, 300, 700]}', '{"text": "STATION 2"}'],
                    [NavStep("find station 2", "inspection")], label="STATION 2")
    d = nav.decide(np.array([-12.0, -5.6]), -math.pi/2, rgb, cam, intr)
    assert d.action == "approach" and d.plate == "STATION 2"
    assert 'reads "STATION 3"' in nav.client.prompts[2]


def test_unlabelled_target_gets_a_second_look(roadmap):
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"action": "approach", "bbox_2d": [400, 300, 600, 700]}',
                              '{"what": "a yellow floor line", "is_target": false}',
                              '{"action": "turn", "turn": "left"}'], [NavStep("find the conveyor", "assembly_line")])
    d = nav.decide(np.array([12.0, 5.0]), 0.0, rgb, cam, intr)
    assert d.action == "turn"
    assert "yellow floor line" in nav.client.prompts[2]


def test_turning_in_place_is_limited(roadmap):
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"action": "turn", "turn": "left"}']*3+['{"action": "turn", "turn": "left"}']*3,
                    [NavStep("find it", None)])
    xy = np.array([0.0, 0.0])
    for _ in range(3):
        assert nav.decide(xy, 0.0, rgb, cam, intr).action == "turn"
    d = nav.decide(xy, 0.0, rgb, cam, intr)  # three more 'turn' replies are rejected -> forced move
    assert d.action == "move" and "moving on" in d.note


def test_drift_while_turning_does_not_reset_the_turn_limit(roadmap):
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"action": "turn", "turn": "left"}']*6, [NavStep("find it", None)])
    for k in range(3):  # a humanoid turning on the spot drifts; only a move/approach resets the count
        assert nav.decide(np.array([0.6*k, 0.0]), 0.0, rgb, cam, intr).action == "turn"
    assert nav.decide(np.array([1.8, 0.0]), 0.0, rgb, cam, intr).action == "move"


def test_region_steps_complete_on_entry_and_approach_geometry(roadmap):
    nav = navigator(roadmap, [], [NavStep("go to inspection", "inspection"), NavStep("find station 2", "inspection")])
    assert not nav.auto_advance(np.array([0.0, 0.0]))
    assert nav.auto_advance(np.array([-12.0, -5.0])) and nav.task.index == 1
    assert not nav.auto_advance(np.array([-12.0, -5.0]))  # the final step needs vision
    goal = approach_pose((-10.0, -8.2), (-10.0, -7.3), (-10.0, -3.0), standoff=1.4)
    assert goal[1] == pytest.approx(-5.9, abs=0.05) and goal[2] == pytest.approx(-math.pi/2)
    assert already_in_front(goal, (goal[0]+0.1, goal[1]))
    assert region_at(roadmap.regions, (-10.0, -8.0)) == "inspection"


def test_lidar_pattern_and_spawn_noise_are_deterministic():
    a, b = mid360_directions(1000, 3), mid360_directions(1000, 3)
    np.testing.assert_array_equal(a, b)
    el = np.degrees(np.arcsin(a[:, 2]))
    assert el.min() >= -52.01 and el.max() <= 7.01
    assert perturbed_spawn([1, 2, 0], 0) == [1.0, 2.0, 0.0]
    x, y, yaw = perturbed_spawn([1, 2, 0], 7)
    assert math.hypot(x-1, y-2) <= 0.3 and abs(math.degrees(yaw)) <= 20
    assert perturbed_spawn([1, 2, 0], 7) == [x, y, yaw]


def test_look_around_once_on_entering_the_target_region(roadmap):
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"seen": "workbenches; conveyor with STATION plate; wall; aisle", "view": 2, '
                              '"target_visible": true}'],
                    [NavStep("go to the assembly line", "assembly_line"),
                     NavStep("find the conveyor and stand in front of it", "assembly_line")])
    inside = np.array([10.2, 3.1])
    assert not nav.needs_scan(inside)            # the region step is still current
    assert nav.auto_advance(inside) and nav.needs_scan(inside)
    views = [(rgb, yaw) for yaw in (0.2, 0.2+math.pi/2, 0.2+math.pi, 0.2-math.pi/2)]
    d = nav.choose_view(inside, views)
    assert d.action == "turn" and d.goal[2] == pytest.approx(0.2+math.pi/2)
    assert "view 2" in d.note and d.prompt_image.shape[1] == 640 and len(nav.client.prompts) == 1
    assert not nav.needs_scan(inside)            # once per step
    assert "turned to view 2" in nav.prompt(inside, 0.2, [])


def test_look_around_falls_back_to_the_current_view_on_invalid_replies(roadmap):
    rgb, _, _ = frame()
    nav = navigator(roadmap, ['{"view": 7}', 'no json'], [NavStep("find it", "inspection")])
    d = nav.choose_view(np.array([-12.0, -5.0]), [(rgb, 0.0), (rgb, 1.0), (rgb, 2.0), (rgb, 3.0)])
    assert d.action == "turn" and d.goal[2] == 0.0 and "continuing from view 1" in d.note


def test_look_around_server_error_keeps_navigating(roadmap):
    rgb, _, _ = frame()
    nav = navigator(roadmap, [], [NavStep("find it", "inspection")])

    def refuse(*args, **kwargs):
        raise OSError("HTTP Error 400: request exceeds the available context size")
    nav.client.chat = refuse
    d = nav.choose_view(np.array([-12.0, -5.0]), [(rgb, 0.5), (rgb, 2.0), (rgb, 3.6), (rgb, 5.1)])
    assert d.action == "turn" and d.goal[2] == 0.5 and "look-around call failed" in d.note


def test_approach_keeps_the_standoff_from_the_object_outline_on_the_map(roadmap):
    # Measured case: the depth on STATION 2's table top was ~0.9 m inside its outline; the robot stopped
    # 0.44 m from the edge and the close-up looked like bare floor to the VLM.
    occupied = roadmap.costmap.occupied
    target, near, robot = (-9.863, -8.077), (-9.4, -7.6), (-6.34, -4.12)
    assert occupied(target) and not occupied(robot)
    x, y, yaw = approach_pose(target, near, robot, standoff=1.4, occupied=occupied)
    edge_gap = y+7.3                                      # station outline: y <= -7.3
    assert 0.9 < edge_gap < 1.3 and not occupied((x, y))
    old = approach_pose(target, near, robot, standoff=1.4)  # depth-only edge, for comparison
    assert old[1]-(-7.3) < edge_gap


@pytest.mark.parametrize("reply, steps, target, label", [
    # a guessed rack number is dropped from the label, the target and the step text
    ('{"steps": [{"goal": "go to the control room", "region": "office_control"}, {"goal": "find server rack 1 '
     'and stand in front of it", "region": "office_control"}], "target": "server rack 1", '
     '"target_label": "SERVER RACK 1", "question": "?", "language": "ko"}',
     ["go to the control room", "find server rack and stand in front of it"], "server rack", ""),
    # the article "a" is not a plate letter
    ('{"steps": [{"goal": "find a conveyor and stand in front of it", "region": "assembly_line"}], '
     '"target": "conveyor A", "target_label": "CONVEYOR A", "question": "?", "language": "ko"}',
     ["find a conveyor and stand in front of it"], "conveyor", ""),
])
def test_decomposition_drops_guessed_identifiers(reply, steps, target, label):
    from g1_factory.vln_navigator import decompose
    config = load_config(ROOT/"configs/factory.yaml")
    regions = {z["id"]: z["bounds"] for z in config["zones"]}
    task, _ = decompose(FakeVLM([reply]), "사무동 관제실에 가서 서버 랙 표시등이 켜져 있는지 확인해", regions, {})
    assert [s.goal for s in task.steps] == steps and task.target == target and task.target_label == label


def test_decomposition_requires_a_final_find_step():
    from g1_factory.vln_navigator import decompose
    config = load_config(ROOT/"configs/factory.yaml")
    regions = {z["id"]: z["bounds"] for z in config["zones"]}
    one_step = ('{"steps": [{"goal": "Go to the charging area", "region": "charging"}], "target": '
                '"the free charging dock (no cart parked)", "target_label": "", "question": "?", "language": "en"}')
    fake = FakeVLM([one_step, one_step])  # asked again once, then the find step is appended
    task, meta = decompose(fake, "Go to the charging area and check whether the free dock is empty.", regions, {})
    assert meta["attempts"] == 2 and "last step must be" in fake.prompts[1]
    assert task.steps[-1].goal.startswith("find the free charging dock") and task.steps[-1].region == "charging"


@pytest.mark.parametrize("label, plate, ok", [
    ("VISION INSPECTION 2", "STATION 2", True), ("STATION 2", "STATION 3", False),
    ("CHARGER 3", "CHARGER 3", True), ("CONVEYOR A", "", False)])
def test_plate_check_compares_identifiers(label, plate, ok):
    from g1_factory.vln_navigator import plate_matches
    assert plate_matches(label, plate) is ok


def test_last_step_needs_an_approach_not_a_stop_from_afar(roadmap):
    # Isaac Sim + 8B: the VLM saw CONVEYOR A and answered "stop" 4 m away; the task is to stand in front of it.
    rgb, cam, intr = frame()
    nav = navigator(roadmap, ['{"action": "stop", "thought": "CONVEYOR A with boxes is in view"}',
                              '{"action": "approach", "bbox_2d": [300, 300, 700, 700]}',
                              '{"what": "a conveyor belt with boxes", "is_target": true}'],
                    [NavStep("go to the assembly line", "assembly_line"),
                     NavStep("find the conveyor and stand in front of it", "assembly_line")])
    nav.task.index = 1
    d = nav.decide(np.array([10.1, 3.4]), math.pi/2, rgb, cam, intr)
    assert d.action == "approach" and "do not stop here" in nav.client.prompts[1]


def test_waypoints_are_offered_when_the_robot_brushed_an_obstacle(roadmap):
    # Measured: the robot stopped touching factory_column_5 at (22.46, 15.18) and got "no reachable waypoint".
    xy = (22.46, 15.18)
    assert roadmap.costmap.distance[roadmap.costmap.grid.world_to_cell(np.array(xy))] < 0.45
    assert roadmap.candidates(xy, 0.13)
