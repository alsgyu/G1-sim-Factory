import json

import pytest
import yaml

from conftest import HERE
from naive_pp.grounding import GroundingError

TASKS = yaml.safe_load((HERE/"configs/instructions.yaml").read_text(encoding="utf-8"))["tasks"]


def payload(task):
    text = task["instruction"]
    return text if isinstance(text, str) else json.dumps(text, ensure_ascii=False)


@pytest.mark.parametrize("name", sorted(TASKS))
def test_every_listed_instruction_names_the_expected_place(campus, name):
    g = campus.ground(payload(TASKS[name]))
    assert g.ok, g.error
    place = g.target.id if g.target.kind == "place" else campus.pkg.place_at(g.target.xy)
    assert place == TASKS[name]["expect"]


def test_the_colleagues_instruction(campus):
    g = campus.ground("Go to the operations control room and check whether the status indicator lights on the "
                      "server rack are on.")
    assert (g.target.kind, g.target.id, g.target.match) == ("place", "office_control", "exact")
    assert g.target.matched == "operations control room" and not g.waypoints


def test_what_to_check_never_moves_a_named_place(campus):
    # "server rack" is a keyword of the control room; the named meeting room wins.
    assert campus.ground("Go to the meeting room and look at the server rack").target.id == "office_meeting"
    assert campus.ground("Check the vending machine in the pump room").target.id == "pump_room"


def test_korean_particles_and_missing_spaces(campus):
    assert campus.ground("조립라인으로 가").target.id == "assembly_line"
    assert campus.ground("펌프실까지 걸어가서 멈춰").target.id == "pump_room"
    assert campus.ground("관제실로").target.id == "office_control"


def test_partial_names(campus):
    g = campus.ground("walk along to the main aisle")
    assert (g.target.id, g.target.match) == ("warehouse_aisle", "partial")
    g = campus.ground("go to the office")
    assert (g.target.id, g.target.match) == ("office_corridor", "partial")


def test_head_noun_variants_and_plurals(campus):
    assert campus.ground("go to the charging zone").target.id == "charging"
    assert campus.ground("walk to the meeting area").target.id == "office_meeting"
    assert campus.ground("go to the engineering workstation").target.id == "office_workstations"


def test_start_via_and_destination_roles(campus):
    g = campus.ground("From the pump room, go through the warehouse-office bridge to the assembly line.")
    assert g.target.id == "assembly_line"
    assert [w.id for w in g.waypoints] == ["warehouse_office_bridge"]
    g = campus.ground("휴게실에서 출발해서 연결 통로를 거쳐 자재 창고로 가")
    assert g.target.id == "material_storage" and [w.id for w in g.waypoints] == ["bridge"]
    g = campus.ground("회의실에서 서버를 확인해")  # 에서 with no other destination: the place itself
    assert g.target.id == "office_meeting"


def test_sequence_of_destinations_ends_at_the_last(campus):
    g = campus.ground("Go to the pump room passage and then to the pump room.")
    assert g.target.id == "pump_room" and [w.id for w in g.waypoints] == ["pump_passage"]


def test_anchor_beats_its_room_when_named_in_full(campus):
    g = campus.ground("Go to the pump room entrance")
    assert (g.target.kind, g.target.id) == ("anchor", "pump_room_entrance")
    assert g.target.xy == (51.85, -19.95)


def test_an_annotated_anchor_inside_the_named_room_is_the_goal(tmp_path):
    from conftest import two_rooms
    from naive_pp.planner import NaivePlanner
    pp = NaivePlanner(two_rooms(tmp_path))  # anchor "B corner" lies in Storage B
    g = pp.ground("Go to storage B and check the shelf at the B corner.")
    assert (g.target.kind, g.target.id, g.target.place) == ("anchor", "b_corner", "room_b")
    g = pp.ground("Go to room A and report what you see.")
    assert (g.target.kind, g.target.id) == ("place", "room_a")


def test_no_place_is_an_error_with_hints(campus):
    g = campus.ground("Check whether the lights are on.")
    assert not g.ok and "known:" in g.error
    g = campus.ground("Go to the meating room")
    assert not g.ok and "Meeting room" in g.alternatives


def test_cancel(campus):
    for text in ("stop", "Cancel", "취소", '{"cancel": true}'):
        assert campus.ground(text).cancel


def test_json_forms(campus):
    g = campus.ground('{"task_id": "a", "place": "Pump room"}')
    assert (g.task_id, g.target.id, g.target.match) == ("a", "pump_room", "id")
    g = campus.ground('{"task_id": "b", "goal": {"x": 45.0, "y": -9.5, "yaw": 1.0}}')
    assert (g.target.kind, g.target.xy, g.target.yaw) == ("pose", (45.0, -9.5), 1.0)
    g = campus.ground('{"subgoals": ["bridge", "office_lounge"]}')
    assert g.target.id == "office_lounge" and [w.id for w in g.waypoints] == ["bridge"]
    g = campus.ground('{"instruction": "펌프실로 가"}')
    assert g.target.id == "pump_room"
    with pytest.raises(GroundingError):
        campus.ground('{"place": "boiler room"}')
    with pytest.raises(GroundingError):
        campus.ground('{"goal": {"x": "nan", "y": 0}}')
