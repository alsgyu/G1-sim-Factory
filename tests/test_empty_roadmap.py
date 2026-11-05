"""Reject invalid deployment maps before querying the VLM or selecting a node."""
import numpy as np
import pytest

from g1_factory.planning import CostMap, OccupancyGrid, OCCUPIED, UNKNOWN
from g1_factory.roadmap import RoadMap
from g1_factory.vln_navigator import NavStep, NavTask, VLNNavigator


class NoVLM:
    def chat(self, *args, **kwargs):
        raise AssertionError('invalid map must fail before a VLM call')


@pytest.mark.parametrize('value', [UNKNOWN, OCCUPIED])
def test_empty_map_has_no_candidates(value):
    grid = OccupancyGrid((0, 0), .1, np.full((30, 30), value, np.int8))
    road = RoadMap(CostMap(grid), {'room': (0, 0, 3, 3)})
    assert road.candidates((1, 1), 0, region='room') == []


@pytest.mark.parametrize('value', [UNKNOWN, OCCUPIED])
def test_navigator_reports_invalid_map_before_inference(value):
    grid = OccupancyGrid((0, 0), .1, np.full((30, 30), value, np.int8))
    road = RoadMap(CostMap(grid), {'room': (0, 0, 3, 3)})
    nav = VLNNavigator(NoVLM(), road, {'room': 'room'}, {'room': (1, 1)})
    nav.start(NavTask('inspect', [NavStep('find the target', 'room')], 'target', '?'))
    decision = nav.decide((1, 1), 0, None, None, None)
    assert decision.action == 'failed'
    assert 'no traversable waypoints' in decision.note
