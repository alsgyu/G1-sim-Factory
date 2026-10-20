import math

import numpy as np
import pytest

from g1_factory import localization as L
from g1_factory import slam_map as S

RES = 0.1


def factory_grid():
    """24 x 16 m hall at 0.1 m: outer walls, a partition with a door, pillars and racks of different sizes."""
    g = np.full((160, 240), S.FREE, np.int8)
    g[[0, -1], :] = S.OCCUPIED
    g[:, [0, -1]] = S.OCCUPIED
    g[:, 120] = S.OCCUPIED
    g[60:75, 120] = S.FREE                      # door in the partition, y 6.0-7.5
    for r, c in ((40, 40), (40, 80), (110, 60), (100, 170), (40, 200)):
        g[r:r+4, c:c+4] = S.OCCUPIED            # pillars 0.4 m
    g[120:128, 140:200] = S.OCCUPIED            # rack 6 x 0.8 m
    g[20:30, 150:158] = S.OCCUPIED              # machine
    g[130:150, 20:24] = S.OCCUPIED              # shelf along the west wall
    return g


def scan(grid, sensor_map, map_from_odom, floor_odom=0.0, rng=None, rays=720, max_range=20.0):
    """Synthetic LiDAR scan in the odom frame: obstacle hits 0.3-1.4 m above the floor plus a floor ring."""
    rng = rng or np.random.default_rng(0)
    sx, sy = sensor_map
    hits = []
    for a in np.linspace(-math.pi, math.pi, rays, endpoint=False):
        for d in np.arange(0.3, max_range, RES/2):
            x, y = sx+d*math.cos(a), sy+d*math.sin(a)
            r, c = int(y/RES), int(x/RES)
            if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
                break
            if grid[r, c] == S.OCCUPIED:
                for z in rng.uniform(0.3, 1.4, 3):
                    hits.append((x+rng.normal(0, 0.01), y+rng.normal(0, 0.01), z))
                break
    ang = rng.uniform(-math.pi, math.pi, 1500)
    rad = rng.uniform(1.2, 6.0, 1500)
    floor = np.c_[sx+rad*np.cos(ang), sy+rad*np.sin(ang), np.zeros(1500)]
    pts = np.r_[np.array(hits), floor]
    odom_from_map = L.inverse(map_from_odom)
    c, s = math.cos(odom_from_map[2]), math.sin(odom_from_map[2])
    xy = pts[:, :2]@np.array([[c, s], [-s, c]])+odom_from_map[:2]
    sensor = L.compose(odom_from_map, (sx, sy, 0.0))
    return np.c_[xy, pts[:, 2]+floor_odom], np.array([sensor[0], sensor[1], floor_odom+1.2])


@pytest.fixture(scope="module")
def grid():
    return factory_grid()


def err(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1]), math.degrees(abs(float(L.wrap(a[2]-b[2]))))


def test_se2_compose_and_inverse_round_trip():
    a, b = (1.0, -2.0, 0.7), (0.3, 4.0, -2.0)
    x, y, yaw = L.compose(L.compose(a, b), L.inverse(b))
    assert np.allclose([x, y, yaw], a)
    assert np.allclose(L.compose(a, L.inverse(a)), (0, 0, 0), atol=1e-12)


def test_distance_field_is_exact_euclidean_within_the_cap():
    rng = np.random.default_rng(2)
    occ = rng.random((40, 50)) < 0.01
    d = L.distance_field(occ, 0.1, 1.0)
    rows, cols = np.nonzero(occ)
    for r, c in [(0, 0), (20, 25), (39, 49), (7, 33)]:
        true = min(np.hypot(rows-r, cols-c).min()*0.1, 1.0)
        assert d[r, c] == pytest.approx(true, abs=1e-5)
    assert (L.distance_field(np.zeros((5, 5), bool), 0.1, 0.5) == 0.5).all()


def test_rough_initial_pose_is_refined_to_the_true_pose_in_a_shifted_odom_frame(grid):
    true_T = (7.5, -3.0, math.radians(35))       # map <- odom: odom started elsewhere, turned
    base_map = (6.0, 9.0, math.radians(-20))
    base_odom = L.compose(L.inverse(true_T), base_map)
    xyz, sensor = scan(grid, base_map[:2], true_T, floor_odom=0.35)
    loc = L.Localizer(grid, (0.0, 0.0), RES)
    loc.initialize((base_map[0]+0.9, base_map[1]-0.8, base_map[2]+math.radians(14)), base_odom)
    u = loc.update(xyz, sensor)
    assert u.state == "localized" and u.initializing
    e, ye = err(loc.map_pose(base_odom), base_map)
    assert e < 0.05 and ye < 0.5
    assert loc.z == pytest.approx(-0.35, abs=0.03)  # map floor at z = 0
    assert u.match.inliers > 0.9


def test_tracking_corrects_odometry_drift_while_walking(grid):
    rng = np.random.default_rng(4)
    path = [(3.0+0.5*k, 7.0, 0.0) for k in range(16)]+[(11.0+0.5*k, 6.7, 0.0) for k in range(1, 12)]
    loc = L.Localizer(grid, (0.0, 0.0), RES)
    true_T = (0.0, 0.0, 0.0)
    loc.initialize(path[0], path[0])
    worst = 0.0
    for k, pose in enumerate(path):
        # odometry drifts: 1.5 deg and 4 cm per step (an odometry that would be 0.9 m / 40 deg off at the end)
        true_T = L.compose((0.04, 0.0, math.radians(1.5)), true_T) if k else true_T
        base_odom = L.compose(L.inverse(true_T), pose)
        xyz, sensor = scan(grid, pose[:2], true_T, rng=rng, rays=360)
        u = loc.update(xyz, sensor)
        assert u.state == "localized", u.message
        worst = max(worst, err(loc.map_pose(base_odom), pose)[0])
    assert worst < 0.06
    assert abs(math.degrees(loc.T[2])) > 35  # the estimate followed the drift


def test_a_guess_far_from_the_robot_is_refused_not_snapped_to_a_wrong_place(grid):
    base = (18.0, 4.0, 0.3)
    xyz, sensor = scan(grid, base[:2], (0.0, 0.0, 0.0))
    loc = L.Localizer(grid, (0.0, 0.0), RES)
    loc.initialize((6.0, 12.0, 0.3), base)  # 14 m off, the other side of the partition
    u = loc.update(xyz, sensor)
    assert u.state == "rejected" and not loc.localized
    assert loc.map_pose(base) is None


def test_the_odometry_prior_holds_the_estimate_along_a_bare_corridor():
    g = np.full((40, 400), S.FREE, np.int8)   # 40 x 4 m corridor: walls only along x
    g[[0, -1], :] = S.OCCUPIED
    loc = L.Localizer(g, (0.0, 0.0), RES, range_m=(0.6, 6.0))
    pose = (20.0, 2.0, 0.0)
    xyz, sensor = scan(g, pose[:2], (0.0, 0.0, 0.0), max_range=6.0)
    loc.T, loc.initializing = (0.1, 0.0, 0.0), False  # tracking, 0.1 m off along the corridor
    u = loc.update(xyz, sensor)
    assert u.state == "localized"
    est = loc.map_pose(pose)
    assert abs(est[1]-pose[1]) < 0.05           # across: fixed by the walls
    assert abs(est[0]-(pose[0]+0.1)) < 0.05     # along: nothing to match, stays at the odometry prediction
    assert u.match.cov[0, 0] > 10*u.match.cov[1, 1]  # and the match says so


def test_no_scan_points_no_fix(grid):
    loc = L.Localizer(grid, (0.0, 0.0), RES)
    assert loc.update(np.zeros((0, 3)), np.zeros(3)).state == "waiting"
    loc.initialize((5.0, 5.0, 0.0), (0.0, 0.0, 0.0))
    u = loc.update(np.zeros((10, 3)), np.array([0.0, 0.0, 1.2]))
    assert u.state == "rejected" and "too few" in u.message


def test_drifting_odometry_scales_distance_and_turns_with_distance_walked():
    from g1_factory.sensors import DriftingOdometry
    odo = DriftingOdometry(scale=0.05, yaw_deg_per_m=1.0)
    assert odo.update(2.0, 1.0, 0.0) == (2.0, 1.0, 0.0)  # starts at the truth
    for k in range(1, 101):
        x, y, yaw = odo.update(2.0+0.1*k, 1.0, 0.0)
    assert math.degrees(yaw) == pytest.approx(10.0, abs=1e-6)       # 10 m walked, 1 deg per metre
    assert math.hypot(x-2.0, y-1.0) == pytest.approx(10.5*np.sinc(10/360), rel=0.01)  # 5 % long, on an arc
    still = DriftingOdometry(0.05, 1.0)
    still.update(0.0, 0.0, 0.3)
    assert still.update(0.0, 0.0, 0.5) == pytest.approx((0.0, 0.0, 0.5))  # turning in place: no drift


def test_a_sudden_odometry_jump_is_searched_again_after_a_few_scans(grid):
    """A slip or a push moves odom at once: rejected while tracking, found again with the initial search window;
    a jump beyond that window is never snapped to a wrong pose (a new /initialpose is needed)."""
    pose = (6.0, 8.0, 0.4)
    for jump, recovers in (((0.8, -0.6, math.radians(10)), True), ((2.0, 1.0, math.radians(25)), False)):
        loc = L.Localizer(grid, (0.0, 0.0), RES)
        loc.initialize(pose, pose)
        assert loc.update(*scan(grid, pose[:2], (0.0, 0.0, 0.0))).state == "localized"
        xyz, sensor = scan(grid, pose[:2], jump)  # odom jumped; the robot did not move
        states = [loc.update(xyz, sensor).state for _ in range(loc.lost_after+2)]
        assert states[:loc.lost_after] == ["rejected"]*loc.lost_after
        e, _ = err(L.compose(loc.T, L.compose(L.inverse(jump), pose)), pose)
        if recovers:
            assert states[-1] == "localized" and e < 0.05
        else:
            assert set(states) == {"rejected"}
