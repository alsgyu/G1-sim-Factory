"""MuJoCo's native viewer: G1 follow camera, episode HUD and route overlay.

This module changes only viewer cameras, decorative geometry and overlays.
RGB/depth cameras used by a navigation policy remain free of debug overlays.
Keyboard callbacks queue events; the simulation thread owns camera/scene edits.
"""
from __future__ import annotations

from queue import Empty, SimpleQueue
import math
import time
from typing import Sequence

import mujoco
import numpy as np

from .minimap import MiniMap


class ViewerDisplay:
    """A small display controller for ``mujoco.viewer.launch_passive``.

    Pass ``key_callback=display.key_callback`` when opening the viewer. Call
    ``process_inputs(viewer, data)`` every loop, including while paused, and
    inspect ``display.paused`` before stepping physics. Call ``update`` before
    ``viewer.sync()``. No additional dashboard process is needed.
    """

    def __init__(self, model, metadata: dict, *, camera_mode: str = "follow"):
        self.model = model
        self.metadata = metadata
        self.paused = False
        if camera_mode not in ("overview", "follow", "ego"):
            raise ValueError("camera_mode must be overview, follow or ego")
        self.camera_mode = camera_mode
        self._events = SimpleQueue()
        self._camera_dirty = True
        self._pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self._minimap = None
        self._last_overlay_update = -math.inf
        self._viewport = None

    def key_callback(self, keycode: int) -> None:
        """GLFW callback: enqueue only, without touching simulation state."""
        if keycode in (ord("1"), ord("2"), ord("3"), ord(" ")):
            self._events.put(keycode)

    def process_inputs(self, viewer, data) -> None:
        while True:
            try:
                keycode = self._events.get_nowait()
            except Empty:
                break
            if keycode == ord(" "):
                self.paused = not self.paused
            else:
                self.camera_mode = {ord("1"): "overview", ord("2"): "follow", ord("3"): "ego"}[keycode]
                self._camera_dirty = True
        if self._camera_dirty:
            with viewer.lock():
                self._configure_camera(viewer, data)
            self._camera_dirty = False

    def _configure_camera(self, viewer, data) -> None:
        # Number keys can also toggle native geom groups. Keep G1 meshes
        # (group 1) visible when using these keys as camera shortcuts.
        viewer.opt.geomgroup[:] = mujoco.MjvOption().geomgroup
        camera = viewer.cam
        cameras = self.metadata.get("camera_names", {})
        name = cameras.get("ego", "ego_rgb") if self.camera_mode == "ego" else cameras.get("map", "map_camera")
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        if self.camera_mode != "follow" and camera_id >= 0:
            camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            camera.fixedcamid = camera_id
            return
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.fixedcamid = -1
        camera.trackbodyid = -1
        if self.camera_mode == "follow":
            camera.lookat[:] = self._robot_position(data)
            camera.distance = 6.0
            camera.azimuth = 135
            camera.elevation = -28
        else:
            xmin, ymin, xmax, ymax = self.metadata.get("bounds", [-25, -17, 53, 17])
            camera.lookat[:] = [(xmin + xmax) / 2, (ymin + ymax) / 2, 0.7]
            camera.distance = max(xmax - xmin, ymax - ymin) * 1.08
            camera.azimuth = 110
            camera.elevation = -58

    def _robot_position(self, data):
        if self._pelvis >= 0:
            return data.xpos[self._pelvis].copy()
        return np.array([data.qpos[0], data.qpos[1], 0.8])

    @staticmethod
    def _ascii(value, limit=90):
        # MuJoCo's built-in font is ASCII. Semantic IDs remain legible even if
        # a configured human label/instruction uses Korean or another script.
        value = " ".join(str(value).split()).encode("ascii", "replace").decode("ascii")
        return value if len(value) <= limit else value[:limit - 3] + "..."

    def update(self, viewer, data, *, mode: str, scenario: str, zone: str,
               next_target: Sequence[float] | None = None,
               waypoint_index: int = 0, waypoint_count: int = 0,
               last_vln_text: str = "", path=None,
               instruction: str = "", status: str = "running") -> None:
        """Refresh HUD and decorative geometry; call before ``viewer.sync``.

        ``waypoint_index`` is the number reached (zero before reaching the first).
        ``path`` can include the spawn followed by all reference waypoints.
        """
        self.process_inputs(viewer, data)
        robot = self._robot_position(data)
        with viewer.lock():
            if self.camera_mode == "follow":
                viewer.cam.lookat[:] = robot
            self._draw_path(viewer.user_scn, path, next_target)
        viewport = viewer.viewport
        if viewport is None:  # The window may close after the loop's is_running check.
            return
        viewport_key = (viewport.left, viewport.bottom, viewport.width, viewport.height)
        now = time.monotonic()
        # Overlays need only 10 Hz, including while paused. Camera/route updates
        # above retain the simulation's normal display cadence.
        if now - self._last_overlay_update < 0.1 and viewport_key == self._viewport:
            return
        self._last_overlay_update = now
        self._viewport = viewport_key
        target = "--" if next_target is None else f"({float(next_target[0]):.1f}, {float(next_target[1]):.1f}) m"
        distance = "--" if next_target is None else f"{np.linalg.norm(robot[:2] - np.asarray(next_target)[:2]):.2f} m"
        left = "Mode\nScenario\nZone\nSimulation\nWaypoints\nNext target\nTarget distance\nCamera\nState"
        right = "\n".join([
            self._ascii(mode), self._ascii(scenario), self._ascii(zone),
            f"{data.time:.1f} s", f"{waypoint_index} / {waypoint_count}" if waypoint_count else "model actions",
            target, distance, self.camera_mode, "PAUSED" if self.paused else self._ascii(status),
        ])
        bottom = "1 Overview   2 Follow G1   3 Ego   SPACE Pause/resume\nMouse: orbit / pan / zoom   Close: window X"
        if instruction:
            bottom += "\nInstruction: " + self._ascii(instruction, 110)
        if last_vln_text:
            bottom += "\nVLN action: " + self._ascii(last_vln_text, 110)
        # These setters wait for the render thread: never hold viewer.lock here.
        # MuJoCo >=3.5.0 releases the GIL during that wait, so GLFW's Python key
        # callback can run. 3.3.7/3.4.0 can deadlock when an overlay is pending.
        viewer.set_texts([
            (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right),
            (mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, bottom, ""),
        ])
        self._draw_minimap(viewer, data, robot, viewport, path, next_target)

    def _draw_minimap(self, viewer, data, robot, viewport, path, next_target):
        margin = 12
        width = min(360, viewport.width - 2 * margin)
        height = min(230, viewport.height - 2 * margin)
        if width < 240 or height < 180:
            viewer.clear_images()
            return
        if self._minimap is None or (self._minimap.width, self._minimap.height) != (width, height):
            self._minimap = MiniMap(self.metadata, width=width, height=height)
        if self._pelvis >= 0:
            rotation = data.xmat[self._pelvis]
            yaw = math.atan2(rotation[3], rotation[0])
        else:
            w, x, y, z = data.qpos[3:7]
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        frame = self._minimap.render(robot[:2], yaw, path=path, next_target=next_target)
        rectangle = mujoco.MjrRect(
            viewport.left + viewport.width - width - margin,
            viewport.bottom + viewport.height - height - margin,
            width, height,
        )
        viewer.set_images([(rectangle, frame)])

    def _draw_path(self, scene, path, next_target) -> None:
        scene.ngeom = 0
        # An unobstructed first-person image is easier to inspect. The regular
        # model renderer never sees user_scn in any camera mode.
        if self.camera_mode == "ego":
            return
        if path is not None:
            points = np.asarray(path, dtype=float)
            if points.ndim == 2 and points.shape[1] >= 2:
                points = np.c_[points[:, :2], np.full(len(points), 0.035)]
                for start, end in zip(points, points[1:]):
                    if scene.ngeom >= scene.maxgeom or not np.isfinite([start, end]).all():
                        break
                    geom = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array([0.12, 0.78, 0.87, 0.75]))
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.035, start, end)
                    scene.ngeom += 1
        if next_target is not None and scene.ngeom < scene.maxgeom:
            target = np.asarray(next_target, dtype=float)
            if target.size >= 2 and np.isfinite(target[:2]).all():
                mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.array([0.16, 0.16, 0.16]), np.array([*target[:2], 0.20]),
                                   np.eye(3).ravel(), np.array([0.2, 0.95, 0.48, 0.8]))
                scene.ngeom += 1
