"""naive-pp: instruction -> place on the robot's own map -> topological route -> A* path -> G1 walks there.

No VLN and no simulator ground truth: the only world knowledge is a map package made by the robot (SLAM occupancy
grid) and annotated by a person (named rooms, corridors, anchors); the place graph is derived from both.
"""
from __future__ import annotations

import sys
from pathlib import Path

# The parent repository (G1-sim-test) provides the map-package tools and the ROS 2 nodes this project reuses.
REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
