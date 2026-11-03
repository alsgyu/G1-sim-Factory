import io
import json
import urllib.request

from PIL import Image

from naive_pp.minimap import Minimap, serve

INSTRUCTION = "Go to the operations control room and check whether the status indicator lights on the server rack are on."


def accepted(g, task_id="a", t=1.0):
    t_ = g.target
    return {"task_id": task_id, "state": "accepted", "sim_time_s": t, "instruction": g.instruction,
            "target": {"kind": t_.kind, "id": t_.id, "label": t_.label}, "match": t_.match, "matched": t_.matched,
            "waypoints": [w.label for w in g.waypoints]}


def test_rooms_carry_their_names_and_korean_alias(campus):
    view = Minimap(campus.pkg)
    places = {p["id"]: p for p in view.meta["places"]}
    assert set(places) == {p["id"] for p in campus.pkg.places}
    assert places["office_control"]["alias"] == "관제실" and places["office_control"]["kind"] == "room"
    assert places["warehouse_aisle"]["kind"] == "corridor"
    h, w = campus.pkg.grid.shape
    assert Image.open(io.BytesIO(view.png)).size == (w, h)


def test_a_task_from_instruction_to_arrival(campus):
    view = Minimap(campus.pkg)
    g = campus.ground(INSTRUCTION)
    plan = campus.plan((0.0, 0.0, 0.0), g.target, g.waypoints)
    view.set_localization("localized")
    view.on_instruction(INSTRUCTION, 0.5)
    view.on_feedback(accepted(g))
    view.set_plan(plan.points)
    view.on_feedback({"task_id": "a", "state": "planned", "sim_time_s": 1.0, **plan.summary()})
    half = len(plan.points)//2
    for k in range(0, half, 10):
        view.set_pose((*plan.points[k], 0.0), 1.0+k*0.25)
    s = view.snapshot()
    assert s["target"]["place"] == "office_control" and s["state"] == "planned"
    assert s["route"][0] == "material_storage" and s["route"][-1] == "office_control"
    assert len(s["trail"]) > 10 and abs(s["remaining_m"]-plan.length_m/2) < 2.0
    view.set_plan([plan.points[half]])  # naive_pp holding the robot is not a new route
    assert len(view.snapshot()["plan"]) == len(plan.points)
    gx, gy, gyaw = plan.goal
    view.set_pose((gx, gy, gyaw), 170.0)
    view.on_feedback({"task_id": "a", "state": "done", "sim_time_s": 170.0, "place": "Operations control",
                      "in_target_place": True, "goal_distance_m": 0.05})
    s = view.snapshot()
    assert s["place"] == "office_control" and s["result"]["in_target_place"] and s["remaining_m"] is None
    assert s["elapsed_s"] == 169.0
    assert [e["state"] for e in s["events"]] == ["instruction", "accepted", "planned", "done"]
    json.dumps(s)


def test_anchor_targets_light_up_their_room_and_unread_instructions_do_not_stop_a_task(campus):
    view = Minimap(campus.pkg)
    g = campus.ground("Go to the pump room entrance")
    view.on_feedback(accepted(g, "b"))
    assert view.snapshot()["target"]["place"] == campus.pkg.place_at(g.target.xy)
    view.on_feedback({"task_id": None, "state": "failed", "sim_time_s": 2.0, "reason": "no place named"})
    s = view.snapshot()
    assert s["state"] == "accepted" and s["result"] is None and s["events"][-1]["state"] == "failed"


def test_the_page_and_its_data_are_served(campus):
    view = Minimap(campus.pkg)
    server = serve(view, "127.0.0.1", 0)
    url = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert b"<canvas" in urllib.request.urlopen(url+"/").read()
        assert json.loads(urllib.request.urlopen(url+"/meta.json").read())["package"] == "campus_dev"
        assert json.loads(urllib.request.urlopen(url+"/state.json").read())["pose"] is None
        assert urllib.request.urlopen(url+"/map.png").read()[:8] == b"\x89PNG\r\n\x1a\n"
    finally:
        server.shutdown()
