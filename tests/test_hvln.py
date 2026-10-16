"""Hierarchical VLN layers without ROS or a VLM server: grounding, planning, tracking, VLM I/O."""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from g1_factory.grounding import GroundingError, SemanticMap, ground_instruction
from g1_factory.planning import CostMap, OccupancyGrid, PathTracker, path_blocked, plan_path
from g1_factory.scene import build_scene, load_config
from g1_factory.vlm import HierarchicalVLM, VLMCall, extract_json

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def smap():
    return SemanticMap.load(load_config(ROOT/"configs/factory.yaml"))


@pytest.fixture(scope="module")
def campus(tmp_path_factory):
    upstream = ROOT/"external/unitree_rl_gym"
    if not (upstream/"resources/robots/g1_description/g1_12dof.xml").is_file():
        pytest.skip("Official G1 assets absent: run python scripts/fetch_assets.py")
    meta = build_scene(ROOT/"configs/factory.yaml", upstream, tmp_path_factory.mktemp("scene")/"scene.xml")
    return meta, CostMap(OccupancyGrid.from_scene(meta, 0.1))


@pytest.mark.parametrize("text, expected", [
    ("Go to the quality inspection area and stop at its entrance.", ["inspection"]),
    ("Walk to the conveyor in the assembly area, then go to loading dock 4.", ["assembly_conveyor", "loading_dock_4"]),
    ("충전 구역을 지나서 사무실 회의실로 가.", ["charging", "office_meeting"]),
    ("업무실, 휴게실, 관제실 순서로 들른 뒤 회의실에서 멈춰.",
     ["office_workstations", "office_lounge", "office_control", "office_meeting"]),
    ("Go to the meeting room in the office", ["office_meeting"]),
    ("Go to the office.", ["office_entrance"]),
])
def test_language_grounding_orders_named_places(smap, text, expected):
    task = ground_instruction(text, smap)
    assert task.ok and task.parser == "language"
    assert [s.place_id for s in task.subgoals] == expected


def test_grounding_never_invents_destinations(smap):
    task = ground_instruction("go to dock 12", smap)
    assert not task.ok and task.subgoals == []
    assert task.suggestions  # helps the external agent rephrase
    with pytest.raises(GroundingError):
        ground_instruction('{"task_id": "x"}', smap)


def test_structured_agent_subgoals_accept_ids_aliases_and_poses(smap):
    payload = json.dumps({"task_id": "a1", "subgoals": ["inspection", {"place": "충전 도크"}, {"x": 20, "y": -1, "yaw": 0}]})
    task = ground_instruction(payload, smap)
    assert task.ok and task.task_id == "a1"
    assert [s.place_id for s in task.subgoals] == ["inspection", "charging_dock_3", "pose"]
    bad = ground_instruction(json.dumps({"subgoals": ["inspection", "nowhere"]}), smap)
    assert not bad.ok and bad.unresolved == ["nowhere"]


def test_every_semantic_place_is_reachable_with_clearance(campus, smap):
    meta, costmap = campus
    grid = costmap.grid
    for place in smap.places.values():
        row, col = grid.world_to_cell(np.array(place.pose[:2]))
        assert costmap.distance[row, col] >= 0.6, f"{place.id} approach pose too close to an obstacle"
    start = smap.places["material_storage"].pose[:2]
    for place_id in ("server_racks", "loading_dock_1", "vending_machine", "charging_dock_3"):
        path, info = plan_path(costmap, start, smap.places[place_id].pose[:2])
        assert info["length_m"] > 0
        rows, cols = grid.world_to_cell(path)
        assert (costmap.distance[rows, cols] >= costmap.lethal_radius).all()


def test_lidar_obstacle_not_in_map_blocks_path_and_replan_avoids_it(campus):
    meta, costmap = campus
    costmap = CostMap(costmap.grid)  # fresh dynamic layer
    start, goal = (-12.0, 0.0), (12.0, 0.0)
    path, _ = plan_path(costmap, start, goal)
    assert not path_blocked(costmap, path)
    xs, ys = np.meshgrid(np.arange(-0.6, 0.6, 0.05), np.arange(-0.6, 0.6, 0.05))
    obstacle = np.stack([xs.ravel(), ys.ravel()], axis=1)
    assert costmap.mark_points(obstacle, stamp=1.0) > 0
    costmap.update(1.0, (0.0, 0.0))
    assert path_blocked(costmap, path)
    replanned, _ = plan_path(costmap, start, goal)
    assert np.min(np.linalg.norm(replanned, axis=1)) > 0.6+costmap.lethal_radius-0.05
    # Hits expire, restoring the static map.
    costmap.update(100.0, (0.0, 0.0), decay_s=10.0)
    assert not path_blocked(costmap, path)


def test_tracker_corrects_cross_track_with_lateral_velocity_and_aligns_heading():
    tracker = PathTracker()
    tracker.set_path(np.array([[0.0, 0.0], [5.0, 0.0]]), final_yaw=math.pi/2)
    vx, vy, wz = tracker.command((1.0, 0.2), 0.0)  # left of the path, heading along it
    assert vx > 0 and vy < 0 and wz < 0
    vx, vy, wz = tracker.command((1.0, 0.0), math.pi)  # facing backwards: rotate in place
    assert vx == 0 and abs(wz) > 0
    tracker.command((4.95, 0.0), 0.0)
    assert tracker.state == "aligning"
    vx, vy, wz = tracker.command((4.95, 0.0), 0.0)
    assert vx == vy == 0 and wz > 0
    tracker.command((4.95, 0.0), math.pi/2)
    assert tracker.state == "reached"
    np.testing.assert_array_equal(tracker.command((4.95, 0.0), math.pi/2), [0, 0, 0])


def test_extract_json_tolerates_fences_and_thinking():
    assert extract_json('<think>{"x": 1}</think>```json\n{"found": true, "bbox_2d": [1, 2, 3, 4]}\n```') == \
        {"found": True, "bbox_2d": [1, 2, 3, 4]}
    with pytest.raises(ValueError):
        extract_json("no json here")


class FakeClient:
    def __init__(self, replies):
        self.replies, self.prompts = list(replies), []

    def chat(self, system, text, images=(), max_tokens=400):
        self.prompts.append(text)
        return VLMCall(self.replies.pop(0), 0.1, len(text), 10, len(images))


def test_vlm_plan_is_validated_retried_refined_and_falls_back(smap):
    client = FakeClient(['{"subgoals": ["the_moon"]}',
                         '{"subgoals": ["office_control"], "target": "server rack", "question": "LED?"}'])
    task, meta = HierarchicalVLM(client, smap).plan_task("관제실 서버 랙 확인해")
    assert meta["source"] == "vlm" and meta["attempts"] == 2 and "invalid" in client.prompts[1]
    # A zone-only answer is refined to the landmark named in the check text.
    assert [s.place_id for s in task.subgoals] == ["office_control", "server_racks"]
    fallback, meta = HierarchicalVLM(FakeClient(["nope", "still nope"]), smap).plan_task("Go to the lounge")
    assert meta["source"] == "rule_fallback" and [s.place_id for s in fallback.subgoals] == ["office_lounge"]


def test_vlm_boxes_are_normalised_to_pixels_and_validated(smap):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    vlm = HierarchicalVLM(FakeClient(['{"found": true, "bbox_2d": [500, 250, 750, 500]}',
                                      '{"found": true, "bbox_2d": [10, 10, 1200, 20]}',
                                      '{"found": false}']), smap)
    box, _ = vlm.locate(image, "rack")
    assert box == pytest.approx((320, 120, 480, 240))
    assert vlm.locate(image, "rack")[0] is None
    assert vlm.locate(image, "rack")[0] is None
