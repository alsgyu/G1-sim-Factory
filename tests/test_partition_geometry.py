"""Metric checks on the same static grid used by the ROS navigator."""
import math
import unittest

import numpy as np

from g1_factory.planning import CostMap, FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from g1_factory.vln_navigator import behind_partition


class PartitionGeometryTest(unittest.TestCase):
    def test_grid_origin_shifts(self):
        for resolution in (0.05, 0.1):
            for millimetres in range(-9, 10):
                with self.subTest(resolution=resolution, millimetres=millimetres):
                    origin = millimetres / 1000
                    centers = origin + (np.arange(round(12 / resolution)) + 0.5) * resolution
                    data = np.zeros((round(2 / resolution), len(centers)), dtype=np.int8)
                    data[:, (centers >= 5.90) & (centers < 6.10)] = OCCUPIED
                    grid = OccupancyGrid((origin, 0), resolution, data)
                    costmap = CostMap(grid)
                    self.assertTrue(behind_partition((6, 1), (1, 1), costmap.occupied, grid=grid))

    def test_oblique_crossings_from_either_side(self):
        resolution = 0.1
        grid = OccupancyGrid((0, 0), resolution, np.zeros((100, 100), dtype=np.int8))
        grid.data[:, 49:51] = OCCUPIED
        costmap = CostMap(grid)
        target = np.array([5., 5.])
        for degrees in (0, 30, 45, 54, 56, 60, 120, 135, 180, 225, 300, 330):
            angle = math.radians(degrees)
            robot = target - 3 * np.array([math.cos(angle), math.sin(angle)])
            crossed = 0.2 / abs(math.cos(angle))
            with self.subTest(degrees=degrees):
                self.assertEqual(behind_partition(target, robot, costmap.occupied, grid=grid), crossed <= 0.35)

    def test_vertical_rays_and_wall_thickness_threshold(self):
        grid = OccupancyGrid((-2, -2), 0.05, np.zeros((80, 80), dtype=np.int8))
        costmap = CostMap(grid)
        for depth, expected in ((7, True), (8, False)):
            grid.data[:] = FREE
            grid.data[36:36 + depth, :] = OCCUPIED
            for y in (-1.5, 1.5):
                with self.subTest(depth=depth, robot_y=y):
                    self.assertEqual(behind_partition((0, -0.05), (0, y), costmap.occupied, grid=grid), expected)

    def test_requires_enough_known_free_space_behind(self):
        grid = OccupancyGrid((0, 0), 0.1, np.zeros((30, 50), dtype=np.int8))
        grid.data[:, 10:12] = OCCUPIED
        costmap = CostMap(grid)
        for idx, value, expected in ((17, OCCUPIED, False), (17, UNKNOWN, False), (18, OCCUPIED, True)):
            with self.subTest(idx=idx, value=value):
                grid.data[:, idx] = value
                self.assertEqual(behind_partition((1.05, 1), (.5, 1), costmap.occupied, grid=grid), expected)
                grid.data[:, idx] = FREE
        grid.data[:, -5:-3] = OCCUPIED
        self.assertFalse(behind_partition((4.55, 1), (4, 1), costmap.occupied, grid=grid))
        self.assertFalse(behind_partition((3, 1), (2, 1), costmap.occupied, grid=grid))
        self.assertFalse(behind_partition((1.05, 1), (1.05, 1), costmap.occupied, grid=grid))

    def test_grid_corner_touch_has_zero_wall_thickness(self):
        grid = OccupancyGrid((0, 0), 0.1, np.zeros((50, 50), dtype=np.int8))
        grid.data[20, 20] = OCCUPIED
        costmap = CostMap(grid)
        self.assertTrue(costmap.occupied((2, 2)))
        self.assertFalse(behind_partition((2, 2), (1, 3), costmap.occupied, grid=grid))
