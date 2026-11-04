"""The Isaac Sim SLAM map with annotations carried over from the MuJoCo map (scripts/transfer_annotations.py)."""
import json
import math
import sys

import numpy as np
import pytest
import yaml

from conftest import CAMPUS, HERE, KEYWORDS
from naive_pp.grounding import load_keywords
from naive_pp.package import FREE, load_package
from naive_pp.planner import NaivePlanner
from g1_factory import slam_map as S

sys.path.insert(0, str(HERE/"scripts"))
from transfer_annotations import register  # noqa: E402

ISAAC = HERE/"maps/campus_isaac"
TASKS = yaml.safe_load((HERE/"configs/instructions.yaml").read_text(encoding="utf-8"))["tasks"]


@pytest.fixture(scope="module")
def isaac():
    return NaivePlanner(load_package(ISAAC), keywords=load_keywords(KEYWORDS))


@pytest.fixture(scope="module")
def isaac_from_dev():
    return register(S.read_map_yaml(CAMPUS/"map.yaml"), S.read_map_yaml(ISAAC/"map.yaml"))


def test_registration_matches_the_simulators_frames(isaac_from_dev):
    # Evaluation only: both packages know where their map frame sits in the simulator (evaluation.json).
    x, y, yaw, inliers, _ = isaac_from_dev
    dev = json.loads((CAMPUS/"evaluation.json").read_text())["world_from_map"]
    isa = json.loads((ISAAC/"evaluation.json").read_text())["world_from_map"]
    c, s = math.cos(-isa["yaw_rad"]), math.sin(-isa["yaw_rad"])
    dx, dy = dev["x"]-isa["x"], dev["y"]-isa["y"]
    assert inliers > 0.95
    assert math.hypot(x-(c*dx-s*dy), y-(s*dx+c*dy)) < 0.1
    assert abs(math.degrees(yaw-(dev["yaw_rad"]-isa["yaw_rad"]))) < 0.5


@pytest.mark.parametrize("name", sorted(TASKS))
def test_every_instruction_plans_on_the_isaac_map(isaac, isaac_from_dev, name):
    x, y, yaw = isaac_from_dev[:3]
    sx, sy, syaw = TASKS[name]["start"]
    start = (x+math.cos(yaw)*sx-math.sin(yaw)*sy, y+math.sin(yaw)*sx+math.cos(yaw)*sy, syaw+yaw)
    text = TASKS[name]["instruction"]
    g = isaac.ground(text if isinstance(text, str) else json.dumps(text))
    if g.target.kind == "pose":  # a pose is in the frame of the map it was given for
        px, py = g.target.xy
        g.target.xy = (x+math.cos(yaw)*px-math.sin(yaw)*py, y+math.sin(yaw)*px+math.cos(yaw)*py)
    plan = isaac.plan(start, g.target, g.waypoints)
    assert isaac.pkg.place_at(plan.goal[:2]) == TASKS[name]["expect"]
    r, c = isaac.costmap.world_to_cell(plan.points)
    assert (isaac.pkg.grid[r, c] == FREE).all()
    assert isaac.costmap.static[r, c].min() >= isaac.cfg.robot_radius-1e-6
    assert np.isfinite(plan.length_m)
