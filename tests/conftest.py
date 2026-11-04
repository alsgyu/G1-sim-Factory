import json
from pathlib import Path
import sys

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from naive_pp.grounding import load_keywords  # noqa: E402
from naive_pp.package import FREE, OCCUPIED, load_package  # noqa: E402
from naive_pp.planner import NaivePlanner  # noqa: E402
from g1_factory import slam_map as S  # noqa: E402

CAMPUS = HERE/"maps/campus_dev"
KEYWORDS = HERE/"configs/place_keywords.yaml"


@pytest.fixture(scope="session")
def campus():
    return NaivePlanner(load_package(CAMPUS), keywords=load_keywords(KEYWORDS))


def two_rooms(folder: Path, doors=((4.0, 5.8),), res=0.1):
    """20 x 10 m: room A (x < 9) and room B (x > 9.2) behind a wall with door gaps (y ranges), walls all round."""
    h, w = int(10/res), int(20/res)
    grid = np.full((h, w), FREE, np.int8)
    grid[:2, :] = grid[-2:, :] = OCCUPIED
    grid[:, :2] = grid[:, -2:] = OCCUPIED
    grid[:, 90:92] = OCCUPIED
    for y0, y1 in doors:
        grid[int(y0/res):int(y1/res), 90:92] = FREE
    S.write_map_yaml(folder, grid, (0.0, 0.0), res)
    (folder/"annotations.json").write_text(json.dumps({"zones": [
        {"id": "room_a", "label": "Room A", "aliases": ["에이방"], "kind": "room", "rects": [[0.2, 0.2, 8.9, 9.8]]},
        {"id": "room_b", "label": "Storage B", "aliases": [], "kind": "room", "rects": [[9.3, 0.2, 19.8, 9.8]]}],
        "anchors": [{"label": "B corner", "xy": [18.5, 1.5]}]}))
    return load_package(folder)
