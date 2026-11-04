"""naive-pp: instruction -> place on the robot's own map -> topological route -> A* path -> G1 walks there.

No VLN and no simulator ground truth: the only world knowledge is a map package made by the robot (SLAM occupancy
grid) and annotated by a person (named rooms, corridors, anchors); the place graph is derived from both.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

# G1-sim-Factory provides the map-package tools and the ROS 2 nodes this project reuses (localizer, path_follower,
# MuJoCo g1_sim): the submodule of this repository, or another checkout named by G1_SIM_ROOT.
ROOT = Path(__file__).resolve().parents[1]
SIM = Path(os.environ.get("G1_SIM_ROOT") or ROOT/"G1-sim-Factory").resolve()
if not (SIM/"g1_factory").is_dir():
    raise ImportError(f"G1-sim-Factory not found at {SIM}: run `git submodule update --init` in {ROOT}, "
                      "or set G1_SIM_ROOT to a G1-sim-Factory checkout")
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))
