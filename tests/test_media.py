import json
import math
from pathlib import Path
import sys
import zlib

import numpy as np
import pytest

from g1_factory import media

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import make_videos  # noqa: E402

META = {"warehouse_bounds": [-24, -16, 24, 16], "office_bounds": [28, -12, 52, 12],
        "utility_bounds": [34.0, -26.0, 46.0, -16.0]}


def test_look_at_is_a_rotation_with_forward_first():
    r = media.look_at([0, -10, 8], [0, 0, 0])
    assert np.allclose(r.T@r, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(r), 1.0)
    assert np.allclose(r[:, 0], np.array([0, 10, -8])/math.hypot(10, 8))
    assert r[2, 2] > 0                                  # image up points to the sky
    down = media.look_at([5, 5, 50], [5, 5, 0])         # straight down: north up, east right
    assert np.allclose(down[:, 0], [0, 0, -1]) and np.allclose(down[:, 2], [0, 1, 0])
    assert np.allclose(down[:, 1], [-1, 0, 0])          # left = west


def test_campus_view_bounds_include_the_pump_room():
    assert media.view_bounds(META, "campus") == [-24, -26.0, 52, 16]
    assert media.view_bounds(META, "utility_bounds") == META["utility_bounds"]


@pytest.mark.parametrize("name", list(media.VIEWS))
def test_fit_view_frames_every_corner_centred(name):
    region, azimuth, elevation, hfov = media.VIEWS[name]
    bounds = media.view_bounds(META, region)
    eye, target = media.fit_view(bounds, azimuth, elevation, hfov, 16/9)
    rotation = media.look_at(eye, target)
    x0, y0, x1, y1 = bounds
    corners = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (0, media.WALL_TOP_M)])
    u, v, depth = media.project(corners, eye, rotation, math.radians(hfov), 16/9)
    assert depth.min() > 0
    assert np.abs(u).max() <= 0.961 and np.abs(v).max() <= 0.961         # inside the 4 % margin
    assert max(np.abs(u).max(), np.abs(v).max()) > 0.94                # and not needlessly far away
    assert abs(u.max()+u.min()) < 0.02 and abs(v.max()+v.min()) < 0.02  # centred
    look = media.look_at(eye, target)[:, 0]
    assert math.isclose(math.degrees(math.asin(-look[2])), elevation, abs_tol=0.5)
    if elevation < 89:
        assert math.isclose(math.degrees(math.atan2(look[1], look[0])), azimuth, abs_tol=0.5)


def test_campus_view_has_the_pump_room_in_front_of_the_office():
    region, azimuth, elevation, hfov = media.VIEWS["campus"]
    eye, target = media.fit_view(media.view_bounds(META, region), azimuth, elevation, hfov, 16/9)
    r = media.look_at(eye, target)
    pump, office, warehouse = (np.array([[40, -21, 0]]), np.array([[40, 0, 0]]), np.array([[0, 0, 0]]))
    up = lambda p: media.project(p, eye, r, math.radians(hfov), 16/9)[1][0]  # noqa: E731
    right = lambda p: media.project(p, eye, r, math.radians(hfov), 16/9)[0][0]  # noqa: E731
    assert up(pump) < up(office)                       # pump room below (nearer than) the office in the image
    assert right(warehouse) < right(office)            # warehouse left, office right


def test_chase_camera_stays_behind_and_above_and_smooths():
    cam = media.ChaseCamera(back=2.8, height=4.6, ahead=1.5, target_dz=-0.4, smoothing=0.85)
    east = np.eye(3)
    eye, target = cam.update([10, 0, 0.75], east)
    assert np.allclose(eye, [7.2, 0, 5.35]) and np.allclose(target, [11.5, 0, 0.35])
    pitch = math.degrees(math.atan2(eye[2]-target[2], target[0]-eye[0]))
    assert 45 < pitch < 55
    north = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)
    eye2, _ = cam.update([10, 0, 0.75], north)        # robot turns: the camera follows only 15 % per frame
    assert np.allclose(eye2, 0.85*eye+0.15*np.array([10, -2.8, 5.35]))


def test_as_rgb8_and_png_writer_round_trip(tmp_path):
    assert media.as_rgb8(None) is None and media.as_rgb8(np.zeros((0, 4))) is None
    rgba = np.zeros((3, 4, 4), np.float32)
    rgba[..., 0] = 1.0
    rgb = media.as_rgb8(rgba)
    assert rgb.dtype == np.uint8 and rgb.shape == (3, 4, 3) and rgb[0, 0].tolist() == [255, 0, 0]
    image = np.random.default_rng(0).integers(0, 256, (5, 7, 3), dtype=np.uint8)
    path = media.write_png(tmp_path/"x.png", image)
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n" and make_videos.image_size(path) == (7, 5)
    idat = data[data.index(b"IDAT")+4:data.index(b"IEND")-8]
    rows = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(5, 1+21)
    assert (rows[:, 0] == 0).all() and np.array_equal(rows[:, 1:].reshape(5, 7, 3), image)
    pil = pytest.importorskip("PIL.Image")
    assert np.array_equal(np.asarray(pil.open(path)), image)


def feedback(state, task_id="t1", **fields):
    return {"task_id": task_id, "state": state, **fields}


def test_recorder_task_segments(tmp_path):
    rec = media.FrameRecorder(tmp_path, "task", fps=30.0, tail_s=2.0, log=lambda *_: None)
    fpv, tpv = np.zeros((48, 64, 3), np.uint8), np.full((72, 128, 3), 200, np.uint8)
    t = 0.0
    assert not rec.wants_frame(t)                       # nothing before the agent accepts a task
    rec.on_feedback(feedback("accepted", instruction="컨베이어 확인"), 1.0)
    for k in range(30):
        t = 1.0+k/30
        assert rec.wants_frame(t)
        rec.add(t, fpv, tpv)
    rec.on_feedback(feedback("decision", action="move"), 1.5)
    rec.on_feedback(feedback("done", result={"verdict": "yes"}), 2.0)
    while rec.wants_frame(t):                           # 2 s tail after done
        rec.add(t, fpv, tpv)
        t += 1/30
    assert 3.99 < t < 4.1
    seg = tmp_path/"01_t1"
    info = json.loads((seg/"frames.json").read_text())
    assert info["state"] == "done" and info["instruction"] == "컨베이어 확인"
    assert info["final_event"]["result"]["verdict"] == "yes"
    assert info["frames"] == len(list((seg/"fpv").iterdir())) == len(list((seg/"tpv").iterdir()))
    assert info["fpv_size"] == [64, 48] and info["tpv_size"] == [128, 72]
    assert [e["state"] for e in info["events"]] == ["accepted", "decision", "done"]
    # a second task preempted by a third: two more folders, numbering continues
    rec.on_feedback(feedback("accepted", task_id="t2", instruction="a"), 10.0)
    rec.wants_frame(10.0)
    rec.add(10.0, fpv, tpv)
    rec.on_feedback(feedback("accepted", task_id="t3", instruction="b"), 11.0)
    rec.on_feedback(feedback("done", task_id="t2"), 11.5)  # a late event of the old task does not end the new one
    assert rec.wants_frame(20.0)
    rec.add(20.0, fpv, tpv)
    rec.shutdown()
    assert json.loads((tmp_path/"02_t2"/"frames.json").read_text())["state"] == "preempted"
    assert json.loads((tmp_path/"03_t3"/"frames.json").read_text())["state"] == "interrupted"


def test_recorder_always_mode(tmp_path):
    rec = media.FrameRecorder(tmp_path/"run_all", "always", log=lambda *_: None)
    frame = np.zeros((8, 8, 3), np.uint8)
    for k in range(3):
        assert rec.wants_frame(k/30)
        rec.add(k/30, frame, frame)
    rec.on_feedback(feedback("done"), 0.1)
    assert rec.wants_frame(5.0)                         # keeps recording to the end
    rec.shutdown()
    info = json.loads((tmp_path/"run_all"/"run"/"frames.json").read_text())
    assert info["frames"] == 3 and info["state"] == "done"


def test_make_videos_commands_and_missing_metadata(tmp_path):
    seg = tmp_path/"01_t1"
    for view in ("fpv", "tpv"):
        (seg/view).mkdir(parents=True)
    media.write_png(seg/"fpv"/"000000.png", np.zeros((48, 64, 3), np.uint8))
    media.write_png(seg/"tpv"/"000000.png", np.zeros((72, 128, 3), np.uint8))
    assert make_videos.segments(tmp_path) == [seg] and make_videos.segments(seg) == [seg]
    info = make_videos.frame_info(seg)                  # no frames.json: from the files
    assert info == {"fps": 30.0, "ext": ".png", "frames": 1, "state": None, "instruction": "", "complete": False,
                    "tpv_height": 72}
    cmds = make_videos.commands("ffmpeg", seg, info, speed=2.0, crf=20)
    assert set(cmds) == {"fpv.mp4", "tpv.mp4", "side_by_side.mp4"}
    side = cmds["side_by_side.mp4"]
    assert side[side.index("-framerate")+1] == "60" and "[1:v]scale=-2:72[f];[0:v][f]hstack=inputs=2" in " ".join(side)
    assert cmds["fpv.mp4"][-1] == str(seg/"fpv.mp4") and "libx264" in cmds["fpv.mp4"]
