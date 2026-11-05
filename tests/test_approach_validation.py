"""Ablation modes retain goal snapping and isolate the gate from the boundary rule."""
import numpy as np

from g1_factory.approach_validation import MODES, alternatives
from g1_factory.planning import CostMap, OccupancyGrid, OCCUPIED


def test_thin_partition_gate_is_independent_of_standoff_rule():
    grid = OccupancyGrid((.01, 0), .1, np.zeros((30, 120), np.int8))
    grid.data[:, 59:61] = OCCUPIED
    options = alternatives((6, 1), (5.8, 1), (1, 1), CostMap(grid), 1.4)
    assert set(options) == set(MODES)
    assert {m for m in MODES if options[m]['hold']} == {'gate', 'full', 'legacy'}
    for mode in ('depth', 'edge'):
        assert options[mode]['goal'] is not None
        assert options[mode]['snap'] is not None
    assert options['edge']['goal'][0] > options['depth']['goal'][0]


def test_depth_baseline_keeps_existing_traversability_correction():
    grid = OccupancyGrid((0, 0), .1, np.zeros((30, 120), np.int8))
    grid.data[:, 50:70] = OCCUPIED
    cm = CostMap(grid)
    options = alternatives((6.9, 1), (6.7, 1), (1, 1), cm, 1.4)
    for mode in MODES:
        assert not options[mode]['hold']
        assert cm.traversable(*grid.world_to_cell(options[mode]['snap']))
    assert options['depth']['goal'] == options['gate']['goal']
    assert options['edge']['goal'] == options['full']['goal']
    assert options['depth']['snap'] != options['depth']['goal'][:2]


def test_legacy_and_cell_lengths_remain_separate_on_shifted_grid():
    grid = OccupancyGrid((0, 0), .1, np.zeros((30, 120), np.int8))
    grid.data[:, 49:51] = OCCUPIED
    options = alternatives((5, 1), (4.9, 1), (1, 1), CostMap(grid), 1.4)
    assert options['full']['hold']
    assert not options['legacy']['hold']


def test_counterfactual_evaluation_does_not_modify_observation_or_map():
    grid = OccupancyGrid((0, 0), .1, np.zeros((30, 120), np.int8))
    cm = CostMap(grid)
    target, near, robot = (np.array(v) for v in [(6., 1.), (5.8, 1.), (1., 1.)])
    before = [v.copy() for v in (target, near, robot, grid.data, cm.distance)]
    alternatives(target, near, robot, cm, 1.4)
    for actual, expected in zip((target, near, robot, grid.data, cm.distance), before):
        np.testing.assert_array_equal(actual, expected)
