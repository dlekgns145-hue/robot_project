from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


DOCKER_DIR = Path(__file__).resolve().parents[2] / "robot_docker"
sys.path.insert(0, str(DOCKER_DIR))

from frontier_core import (  # noqa: E402
    GridSpec,
    cell_has_obstacle_clearance,
    cluster_frontiers,
    frontier_candidates,
    frontier_cell_indices,
    grid_cell_to_world,
    reachable_free_cell_indices,
    world_to_grid_cell,
)


class FrontierCoreTests(unittest.TestCase):
    def test_known_free_boundary_becomes_one_frontier_cluster(self) -> None:
        spec = GridSpec(width=5, height=5, resolution=0.1)
        data = [-1] * 25
        for y in range(1, 4):
            for x in range(1, 4):
                data[y * spec.width + x] = 0

        cells = frontier_cell_indices(data, spec)
        clusters = cluster_frontiers(cells, spec)

        self.assertEqual(len(cells), 8)
        self.assertEqual(sorted(len(cluster) for cluster in clusters), [8])

    def test_occupied_cells_are_not_frontiers(self) -> None:
        spec = GridSpec(width=3, height=3, resolution=0.1)
        data = [-1] * 9
        data[4] = 100

        self.assertEqual(frontier_cell_indices(data, spec), set())

    def test_separated_free_regions_form_separate_clusters(self) -> None:
        spec = GridSpec(width=10, height=4, resolution=0.1)
        data = [-1] * 40
        for x in (1, 2, 7, 8):
            for y in (1, 2):
                data[y * spec.width + x] = 0

        clusters = cluster_frontiers(frontier_cell_indices(data, spec), spec)

        self.assertEqual(sorted(len(cluster) for cluster in clusters), [4, 4])

    def test_rotated_map_origin_is_respected(self) -> None:
        spec = GridSpec(
            width=1,
            height=1,
            resolution=1.0,
            origin_x=1.0,
            origin_y=2.0,
            origin_yaw=math.pi / 2.0,
        )

        x, y = grid_cell_to_world(0, 0, spec)

        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 2.5)
        self.assertEqual(world_to_grid_cell(x, y, spec), (0, 0))

    def test_unreachable_frontier_is_not_a_candidate(self) -> None:
        spec = GridSpec(width=10, height=5, resolution=0.1)
        data = [-1] * 50
        for y in range(1, 4):
            for x in (1, 2, 7, 8):
                data[y * spec.width + x] = 0

        reachable = reachable_free_cell_indices(
            data,
            spec,
            robot_x=0.15,
            robot_y=0.25,
        )
        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.15,
            robot_y=0.25,
            min_cells=2,
            min_distance=0.0,
            max_distance=10.0,
        )

        self.assertTrue(reachable)
        self.assertTrue(all(index % spec.width < 3 for index in reachable))
        self.assertEqual(len(candidates), 1)
        self.assertLess(candidates[0].grid_x, 3)

    def test_goal_standoff_selects_stable_interior_free_cell(self) -> None:
        spec = GridSpec(width=7, height=7, resolution=0.1)
        data = [-1] * 49
        for y in range(1, 6):
            for x in range(1, 6):
                data[y * spec.width + x] = 0

        frontiers = frontier_cell_indices(data, spec)
        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.35,
            robot_y=0.35,
            min_cells=4,
            min_distance=0.0,
            max_distance=10.0,
            goal_standoff=0.1,
        )

        self.assertEqual(len(candidates), 1)
        goal_index = candidates[0].grid_y * spec.width + candidates[0].grid_x
        self.assertNotIn(goal_index, frontiers)
        self.assertEqual(data[goal_index], 0)

    def test_far_frontier_is_staged_along_connected_free_path(self) -> None:
        spec = GridSpec(width=45, height=5, resolution=0.1)
        data = [-1] * (spec.width * spec.height)
        for x in range(1, 44):
            for y in range(1, 4):
                data[y * spec.width + x] = 0

        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.25,
            robot_y=0.25,
            min_cells=2,
            min_distance=0.45,
            max_distance=7.0,
            goal_standoff=0.1,
            maximum_goal_step_distance=1.25,
        )

        self.assertTrue(candidates)
        self.assertLessEqual(candidates[0].distance, 1.26)
        goal_index = candidates[0].grid_y * spec.width + candidates[0].grid_x
        self.assertEqual(data[goal_index], 0)

    def test_goal_too_close_to_occupied_cell_is_rejected(self) -> None:
        spec = GridSpec(width=9, height=9, resolution=0.1)
        data = [-1] * (spec.width * spec.height)
        for y in range(1, 8):
            for x in range(1, 8):
                data[y * spec.width + x] = 0
        obstacle_index = 4 * spec.width + 6
        data[obstacle_index] = 100

        self.assertFalse(
            cell_has_obstacle_clearance(
                data,
                spec,
                4 * spec.width + 4,
                clearance_m=0.25,
            )
        )
        self.assertTrue(
            cell_has_obstacle_clearance(
                data,
                spec,
                4 * spec.width + 2,
                clearance_m=0.25,
            )
        )

    def test_blacklist_excludes_only_nearby_part_of_cluster(self) -> None:
        spec = GridSpec(width=5, height=5, resolution=0.2)
        data = [-1] * 25
        for y in range(1, 4):
            for x in range(1, 4):
                data[y * spec.width + x] = 0
        initial = frontier_candidates(
            data,
            spec,
            robot_x=-1.0,
            robot_y=-1.0,
            min_cells=4,
            min_distance=0.0,
            max_distance=10.0,
        )
        self.assertEqual(len(initial), 1)

        alternatives = frontier_candidates(
            data,
            spec,
            robot_x=-1.0,
            robot_y=-1.0,
            min_cells=4,
            min_distance=0.0,
            max_distance=10.0,
            blacklisted=[(initial[0].x, initial[0].y)],
            blacklist_radius=0.1,
        )

        self.assertEqual(len(alternatives), 1)
        self.assertGreater(
            math.hypot(
                alternatives[0].x - initial[0].x,
                alternatives[0].y - initial[0].y,
            ),
            0.1,
        )

    def test_invalid_grid_size_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            frontier_cell_indices([0], GridSpec(2, 2, 0.1))


if __name__ == "__main__":
    unittest.main()
