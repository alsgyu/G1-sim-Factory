"""Semantic/topological map layers and the recognition-accuracy ground truth."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from g1_factory.grounding import SemanticMap
from g1_factory.recognition import (INTRINSICS, map_from_optical, match_detections, sample_poses, visible_objects,
                                    zone_of)
from g1_factory.scene import build_scene, load_config
from g1_factory.semantic_world import build_semantic_map, route

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    mujoco = pytest.importorskip("mujoco")
    upstream = ROOT/"external/unitree_rl_gym"
    if not (upstream/"resources/robots/g1_description/g1_12dof.xml").is_file():
        pytest.skip("Official G1 assets absent: run python scripts/fetch_assets.py")
    meta = build_scene(ROOT/"configs/factory.yaml", upstream, tmp_path_factory.mktemp("scene")/"scene.xml")
    model = mujoco.MjModel.from_xml_path(meta["scene_path"])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    config = load_config(ROOT/"configs/factory.yaml")
    anchors = {p.label: p.pose[:2] for p in SemanticMap.load(config).annotated().places.values()
               if p.kind == "landmark"}
    return mujoco, model, data, meta, build_semantic_map(model, data, meta, anchors)


def test_semantic_layer_has_every_plated_object_and_the_pump_room(world):
    semantic = world[4]
    labels = {o["label"] for o in semantic["objects"] if o["label"]}
    assert {"STATION 2", "CHARGER 3", "CONVEYOR A", "SERVER RACK 3", "PUMP P-101", "MOTOR M-101", "TK-101",
            "MCC-1"} <= labels
    pump = next(o for o in semantic["objects"] if o["class"] == "centrifugal pump")
    assert pump["region"] == "pump_room" and pump["aabb"][5] > 1.0
    assert next(o for o in semantic["objects"] if o["label"] == "STATION 2")["region"] == "inspection"


def test_topological_map_connects_every_place_and_routes_through_doors(world):
    semantic = world[4]
    ids = {p["id"] for p in semantic["places"]}
    assert {z["id"] for z in semantic["zones"]} | {"warehouse_aisle", "office_corridor", "bridge", "pump_passage"} == ids
    for place in ids:
        assert any(place in (e["a"], e["b"]) for e in semantic["edges"]), place
    assert route(semantic, "material_storage", "pump_room") == [
        "material_storage", "warehouse_aisle", "bridge", "office_corridor", "pump_passage", "pump_room"]
    door = next(e for e in semantic["edges"] if {e["a"], e["b"]} == {"pump_passage", "pump_room"})["door"]
    assert 38 < door[0] < 42 and -17.5 < door[1] < -15.5


def test_capture_poses_cover_every_place_in_free_space(world):
    meta, semantic = world[3], world[4]
    poses = sample_poses(semantic, meta, 3, seed=1)
    assert {p["place"] for p in poses} == {p["id"] for p in semantic["places"]}
    assert poses == sample_poses(semantic, meta, 3, seed=1)
    assert zone_of(semantic, (40, -21)) == "pump_room" and zone_of(semantic, (0, 0)) == "aisle"


def render(world, x, y, yaw):
    mujoco, model, data = world[:3]
    from g1_factory.locomotion import G1Locomotion
    from g1_factory.sensors import RGBDCamera
    walk = G1Locomotion(model, data, ROOT/"external/unitree_rl_gym")
    walk.reset([x, y], yaw)
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    camera = RGBDCamera(model, "head_rgbd", 640, 480)
    rgb, depth = camera.render(data)
    camera.close()
    return depth, map_from_optical(data.xpos[pelvis], data.xquat[pelvis])


def test_ground_truth_sees_the_pump_skid_and_nothing_through_a_wall(world):
    semantic = world[4]
    depth, cam = render(world, 40.0, -17.6, -math.pi/2)  # pump room door, facing the skid
    seen = {t["class"] for t in visible_objects(semantic["objects"], depth, cam, INTRINSICS) if t["status"] == "visible"}
    assert {"centrifugal pump", "electric motor", "coupling guard"} <= seen
    depth, cam = render(world, 40.0, -17.0, math.pi/2)    # same spot facing the north wall: skid behind the robot
    seen = {t["class"] for t in visible_objects(semantic["objects"], depth, cam, INTRINSICS) if t["status"] == "visible"}
    assert "centrifugal pump" not in seen


def test_detections_are_matched_by_class_position_and_plate(world):
    semantic = world[4]
    objects = {o["id"]: o for o in semantic["objects"]}
    depth, cam = render(world, 40.0, -17.6, -math.pi/2)
    truth = visible_objects(semantic["objects"], depth, cam, INTRINSICS)
    pump = next(t for t in truth if t["class"] == "centrifugal pump")
    motor = next(t for t in truth if t["class"] == "electric motor")
    preds = [{"class": "centrifugal pump", "plate": "PUMP P-101", "bbox_px": pump["bbox_px"]},
             {"class": "pressure tank", "plate": "", "bbox_px": motor["bbox_px"]},      # right place, wrong class
             {"class": "conveyor", "plate": "", "bbox_px": [0, 0, 40, 40]}]             # nothing there
    m = match_detections(preds, truth, objects, depth, cam)
    assert pump["id"] in m["tp"] and m["fp"] == 2
    assert m["wrong_class"] and m["wrong_class"][0]["truth"] == "electric motor"
    assert m["plates"] == [{"id": pump["id"], "truth": "PUMP P-101", "read": "PUMP P-101", "correct": True}]
    assert motor["id"] in m["fn"]


def test_topological_route_reaches_the_vln_prompt(world):
    from g1_factory.semantic_world import topological_hint
    semantic = world[4]
    hint = topological_hint(semantic, (0.0, 0.0), 0.0, "pump_room")   # main aisle crossing, facing east
    assert hint.startswith("Topological route to Pump room: Warehouse main aisle -> Warehouse-office bridge")
    assert "Pump room passage -> Pump room" in hint and "next opening (into Warehouse-office bridge): ahead" in hint
    assert topological_hint(semantic, (40.0, -21.0), 0.0, "pump_room") is None   # already there
