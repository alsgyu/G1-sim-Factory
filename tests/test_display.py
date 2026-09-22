"""Viewer-map regressions without a window, GPU, or robot policy."""

from __future__ import annotations

import copy
import math
from contextlib import contextmanager

import numpy as np
import pytest

from g1_factory.minimap import MiniMap


class CapturingViewer:
    """Native MuJoCo objects behind a window-free viewer protocol stub.

    This checks our locking/API contract, not the native render thread or GLFW.
    """

    def __init__(self, mujoco, model):
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.user_scn = mujoco.MjvScene(model, maxgeom=32)
        self.viewport = mujoco.MjrRect(17, 23, 1000, 700)
        self.locked = False
        self.images = []
        self.texts = []
        self.image_updates = 0
        self.image_clears = 0

    @contextmanager
    def lock(self):
        assert not self.locked, "Viewer lock must not be nested"
        self.locked = True
        try:
            yield
        finally:
            self.locked = False

    def set_images(self, images):
        assert not self.locked, "Blocking overlay setter called inside viewer lock"
        self.images = [(rect, frame.copy()) for rect, frame in images]
        self.image_updates += 1

    def clear_images(self):
        assert not self.locked
        self.images = []
        self.image_clears += 1

    def set_texts(self, texts):
        assert not self.locked, "Blocking overlay setter called inside viewer lock"
        self.texts = texts


@pytest.fixture
def site_metadata():
    # Match the campus coordinate system, including the office bridge at y=0.
    return {
        "bounds": [-25, -17, 53, 17],
        "warehouse_bounds": [-24, -16, 24, 16],
        "office_bounds": [28, -12, 52, 12],
        "config": {"layout": {"corridor_width": 4}},
        "zones": {
            "material_storage": {"kind": "warehouse", "bounds": [-23, 2, -1, 15]},
            "assembly_line": {"kind": "warehouse", "bounds": [1, 2, 23, 15]},
            "inspection": {"kind": "warehouse", "bounds": [-23, -15, -1, -2]},
            "charging": {"kind": "warehouse", "bounds": [1, -15, 23, -2]},
            "office_workstations": {"kind": "office", "bounds": [29, 2, 39, 11]},
            "office_meeting": {"kind": "office", "bounds": [41, 2, 51, 11]},
            "office_lounge": {"kind": "office", "bounds": [29, -11, 39, -2]},
            "office_control": {"kind": "office", "bounds": [41, -11, 51, -2]},
        },
        "obstacles": [{"id": "rack", "bounds": [-22, 11, -18, 14]}],
    }


@pytest.mark.parametrize("size", [(360, 230), (720, 280), (240, 300)])
def test_site_map_preserves_metres_and_world_north(site_metadata, size):
    minimap = MiniMap(site_metadata, width=size[0], height=size[1])
    origin = np.array(minimap.world_to_pixel((0, 0)))
    east = np.array(minimap.world_to_pixel((10, 0))) - origin
    north = np.array(minimap.world_to_pixel((0, 10))) - origin
    assert east[0] > 0 and east[1] == 0
    assert north[0] == 0 and north[1] < 0
    assert np.linalg.norm(east) == pytest.approx(np.linalg.norm(north))
    lower_left = minimap.world_to_pixel((-25, -17))
    upper_right = minimap.world_to_pixel((53, 17))
    assert 0 <= lower_left[0] < upper_right[0] < size[0]
    assert 0 <= upper_right[1] < lower_left[1] < size[1]
    # An office pose must lie to the right of the warehouse on the same map.
    assert minimap.world_to_pixel((40, 0))[0] > minimap.world_to_pixel((0, 0))[0]


def _robot_pixels(frame):
    # The filled gold arrow is distinct from the route, floor plan, and text.
    rows, columns = np.nonzero(np.all(frame == (255, 183, 77), axis=2))
    assert len(rows) > 10
    return np.column_stack((columns, rows))


def test_live_robot_marker_moves_and_previous_marker_is_erased(site_metadata):
    minimap = MiniMap(site_metadata)
    start = minimap.render((0, 0), 0)
    moved = minimap.render((40, 0), 0)
    assert moved.shape == (230, 360, 3)
    assert moved.dtype == np.uint8 and moved.flags.c_contiguous
    actual_shift = _robot_pixels(moved).mean(axis=0) - _robot_pixels(start).mean(axis=0)
    expected_shift = np.array(minimap.world_to_pixel((40, 0))) - minimap.world_to_pixel((0, 0))
    np.testing.assert_allclose(actual_shift, expected_shift, atol=1)
    # A fresh map at this pose must match: no stale robot trail in the cache.
    np.testing.assert_array_equal(moved, MiniMap(site_metadata).render((40, 0), 0))


@pytest.mark.parametrize("yaw", [0, math.pi / 2, math.pi, -math.pi / 2])
def test_heading_arrow_points_in_world_yaw_direction(site_metadata, yaw):
    minimap = MiniMap(site_metadata)
    center = np.array(minimap.world_to_pixel((0, 0)))
    pixels = _robot_pixels(minimap.render((0, 0), yaw)) - center
    image_forward = np.array((math.cos(yaw), -math.sin(yaw)))
    along_heading = pixels @ image_forward
    # Arrow nose extends farther than its tail, including for +Y (image up).
    assert along_heading.max() > 7
    assert along_heading.max() > -along_heading.min() + 1


def test_route_and_target_are_optional_without_polluting_next_frame(site_metadata):
    minimap = MiniMap(site_metadata)
    original = minimap.render((-20, 0), 0)
    route = np.array([[-12, 0], [12, 0], [12, -4]], dtype=float)
    route_copy = route.copy()
    with_route = minimap.render((-20, 0), 0, path=route, next_target=[12, -4])
    # The route and goal are visible at their world locations.
    route_x, route_y = map(round, minimap.world_to_pixel((0, 0)))
    target_x, target_y = map(round, minimap.world_to_pixel((12, -4)))
    assert np.any(with_route[route_y-1:route_y+2, route_x-1:route_x+2] !=
                  original[route_y-1:route_y+2, route_x-1:route_x+2])
    assert np.any(with_route[target_y-2:target_y+3, target_x-2:target_x+3] !=
                  original[target_y-2:target_y+3, target_x-2:target_x+3])
    np.testing.assert_array_equal(route, route_copy)
    np.testing.assert_array_equal(minimap.render((-20, 0), 0), original)


def test_moved_shelf_refreshes_static_map_without_mutating_metadata(site_metadata):
    minimap = MiniMap(site_metadata)
    original_metadata = copy.deepcopy(site_metadata)
    before = minimap.render((0, 0), 0)
    assert site_metadata == original_metadata

    site_metadata["obstacles"][0]["bounds"] = [-22, -14, -18, -11]
    moved_metadata = copy.deepcopy(site_metadata)
    after = minimap.render((0, 0), 0)
    assert site_metadata == moved_metadata
    for center in [(-20, 12.5), (-20, -12.5)]:
        px, py = map(round, minimap.world_to_pixel(center))
        assert not np.array_equal(before[py, px], after[py, px])
    np.testing.assert_array_equal(after, MiniMap(site_metadata).render((0, 0), 0))


@pytest.fixture
def display_fixture(site_metadata, monkeypatch):
    mujoco = pytest.importorskip("mujoco")
    from g1_factory.display import ViewerDisplay

    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <geom type="plane" size="60 30 .1"/>
          <camera name="map_camera" pos="14 0 50"/>
          <body name="pelvis" pos="0 0 .8">
            <freejoint/><geom type="sphere" size=".1" group="1"/>
            <camera name="ego_rgb" pos="0 0 .2"/>
          </body>
        </worldbody></mujoco>
    """)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    site_metadata["camera_names"] = {"map": "map_camera", "ego": "ego_rgb"}
    viewer = CapturingViewer(mujoco, model)
    display = ViewerDisplay(model, site_metadata)
    clock = [1.0]
    monkeypatch.setattr("g1_factory.display.time.monotonic", lambda: clock[0])
    return mujoco, model, data, viewer, display, clock


def _update(display, viewer, data, **kwargs):
    display.update(viewer, data, mode="Fixed path", scenario="test", zone="transit", **kwargs)


def test_camera_keys_are_queued_and_restore_geometry_visibility(display_fixture):
    mujoco, model, data, viewer, display, _ = display_fixture
    display.process_inputs(viewer, data)
    for key, mode, camera in [("1", "overview", "map_camera"),
                              ("2", "follow", None), ("3", "ego", "ego_rgb")]:
        previous_mode = display.camera_mode
        viewer.opt.geomgroup[:] = 0  # Native numeric shortcut toggles visibility.
        display.key_callback(ord(key))
        assert display.camera_mode == previous_mode
        display.process_inputs(viewer, data)
        assert display.camera_mode == mode
        np.testing.assert_array_equal(viewer.opt.geomgroup, mujoco.MjvOption().geomgroup)
        if camera is not None:
            assert viewer.cam.type == mujoco.mjtCamera.mjCAMERA_FIXED
            assert viewer.cam.fixedcamid == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        else:
            assert viewer.cam.type == mujoco.mjtCamera.mjCAMERA_FREE
            np.testing.assert_allclose(viewer.cam.lookat, data.xpos[1])


def test_pause_key_can_be_processed_and_displayed_while_paused(display_fixture):
    _, _, data, viewer, display, clock = display_fixture
    display.key_callback(ord(" "))
    assert not display.paused
    _update(display, viewer, data)
    assert display.paused and "PAUSED" in viewer.texts[0][3]
    assert viewer.images
    display.key_callback(ord("q"))  # Unhandled keys must not change pause state.
    display.process_inputs(viewer, data)
    assert display.paused
    display.key_callback(ord(" "))
    clock[0] += .2
    _update(display, viewer, data)
    assert not display.paused and "PAUSED" not in viewer.texts[0][3]


def test_minimap_anchors_to_actual_viewport_and_resizes_immediately(display_fixture):
    mujoco, _, data, viewer, display, _ = display_fixture
    _update(display, viewer, data)
    rect, image = viewer.images[0]
    assert (rect.left, rect.bottom, rect.width, rect.height) == (645, 481, 360, 230)
    assert image.shape == (230, 360, 3)

    # No clock advance: resize must still invalidate the 10 Hz overlay cache.
    viewer.viewport = mujoco.MjrRect(31, 47, 310, 210)
    _update(display, viewer, data)
    rect, image = viewer.images[0]
    assert (rect.left, rect.bottom, rect.width, rect.height) == (43, 59, 286, 186)
    assert image.shape == (186, 286, 3)
    viewer.viewport = mujoco.MjrRect(31, 47, 230, 160)
    _update(display, viewer, data)
    assert viewer.images == [] and viewer.image_clears == 1


def test_minimap_tracks_native_body_pose_at_bounded_refresh_rate(display_fixture):
    mujoco, model, data, viewer, display, clock = display_fixture
    _update(display, viewer, data)
    first = viewer.images[0][1].copy()
    data.qpos[:2] = [40, 0]
    data.qpos[3:7] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
    mujoco.mj_forward(model, data)
    clock[0] += .05
    _update(display, viewer, data)
    assert viewer.image_updates == 1
    np.testing.assert_allclose(viewer.cam.lookat, data.xpos[1])
    clock[0] += .06
    _update(display, viewer, data)
    assert viewer.image_updates == 2
    second = viewer.images[0][1]
    assert not np.array_equal(first, second)
    expected = MiniMap(display.metadata).render((40, 0), math.pi / 2)
    np.testing.assert_array_equal(second, expected)


def test_ego_hides_path_and_overlays_do_not_mutate_model_scene(display_fixture):
    mujoco, model, data, viewer, display, clock = display_fixture
    qpos_before, geom_pos_before = data.qpos.copy(), model.geom_pos.copy()
    model_geom_count = model.ngeom
    path = [[0, 0], [3, 0], [3, 4]]
    _update(display, viewer, data, path=path, next_target=[3, 4])
    assert viewer.user_scn.ngeom == 3  # Two segments and one target.

    # The ordinary model scene used by policy rendering excludes user_scn.
    policy_scene = mujoco.MjvScene(model, maxgeom=32)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
    camera.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "ego_rgb")
    mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera,
                          mujoco.mjtCatBit.mjCAT_ALL, policy_scene)
    assert policy_scene.ngeom == model_geom_count

    display.key_callback(ord("3"))
    clock[0] += .2
    _update(display, viewer, data, path=path, next_target=[3, 4])
    assert viewer.user_scn.ngeom == 0
    assert viewer.images  # Debug minimap remains in the viewer's ego mode.
    assert model.ngeom == model_geom_count
    np.testing.assert_array_equal(data.qpos, qpos_before)
    np.testing.assert_array_equal(model.geom_pos, geom_pos_before)


@pytest.mark.parametrize("version", ["3.3.7", "3.4.0"])
def test_old_mujoco_gui_is_rejected_before_setup(version, tmp_path, monkeypatch):
    from g1_factory import run

    monkeypatch.setattr(run.mujoco, "__version__", version)

    def unexpected_setup(*args, **kwargs):
        pytest.fail("Unsupported GUI version reached simulation setup")

    monkeypatch.setattr(run, "build_scene", unexpected_setup)
    output = tmp_path / "must_not_be_created"
    with pytest.raises(SystemExit, match=r"GUI requires MuJoCo >=3\.5\.0"):
        run.main(["--output", str(output)])
    assert not output.exists()
