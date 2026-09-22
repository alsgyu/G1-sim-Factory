"""Replace this high-level module without changing the learned walking policy."""
from dataclasses import dataclass
import math
import numpy as np


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w*z + x*y), 1 - 2*(y*y + z*z))


def cross_track_error(position, points):
    p = np.asarray(position, dtype=float)
    pts = np.asarray(points, dtype=float)
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0]))
    a, b = pts[:-1], pts[1:]
    delta = b - a
    frac = np.clip(np.sum((p-a)*delta, axis=1) / np.maximum(np.sum(delta*delta, axis=1), 1e-12), 0, 1)
    return float(np.min(np.linalg.norm(p - (a + frac[:, None]*delta), axis=1)))


@dataclass
class WaypointFollower:
    points: object
    max_speed: float = 0.35
    max_yaw_rate: float = 0.5
    tolerance: float = 0.3
    index: int = 0

    def __post_init__(self):
        self.points = np.asarray(self.points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) == 0:
            raise ValueError("Path must contain at least one [x,y] waypoint")
        if not np.isfinite(self.points).all():
            raise ValueError("Path must be finite")
        if min(self.max_speed, self.max_yaw_rate, self.tolerance) <= 0:
            raise ValueError("Follower limits must be positive")

    @property
    def done(self):
        return self.index >= len(self.points)

    def command(self, xy, yaw):
        xy = np.asarray(xy)
        while not self.done and np.linalg.norm(self.points[self.index]-xy) <= self.tolerance:
            self.index += 1
        if self.done:
            return np.zeros(3, dtype=np.float32)
        delta = self.points[self.index] - xy
        error = wrap_angle(math.atan2(delta[1], delta[0])-yaw)
        speed = min(self.max_speed, float(np.linalg.norm(delta))*0.8)
        # Brake at corners. No collision avoidance or replanning is hidden here.
        speed *= max(0.0, math.cos(error)) if abs(error) < 0.6 else 0.0
        turn = np.clip(error*1.5, -self.max_yaw_rate, self.max_yaw_rate)
        return np.asarray([speed, 0.0, turn], dtype=np.float32)


class RelativeActionFollower:
    """Execute a VLN relative motion with measured pose, not timed teleportation."""
    def __init__(self, action, xy, yaw):
        self.kind = action.kind
        self.start = np.array(xy, dtype=float)
        self.target_yaw = yaw
        self.path = None
        if action.kind == "move_forward":
            goal = self.start + action.value*np.array([math.cos(yaw), math.sin(yaw)])
            self.path = WaypointFollower([goal], tolerance=min(0.15, action.value/3))
        elif action.kind in ("turn_left", "turn_right"):
            self.target_yaw = wrap_angle(yaw+math.radians(action.value)*(1 if action.kind == "turn_left" else -1))
        self.done = action.kind == "stop"

    def command(self, xy, yaw):
        if self.done:
            return np.zeros(3, dtype=np.float32)
        if self.path:
            cmd = self.path.command(xy, yaw)
            self.done = self.path.done
            return cmd
        error = wrap_angle(self.target_yaw-yaw)
        self.done = abs(error) < 0.08
        return np.array([0, 0, 0 if self.done else np.clip(error, -0.4, 0.4)], dtype=np.float32)
