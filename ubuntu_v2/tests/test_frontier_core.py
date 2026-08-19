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
    frontier_distance_weight,
    frontier_goal_step_distance,
    grid_cell_to_world,
    reachable_free_cell_indices,
    unknown_region_size,
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

    def test_far_free_seed_is_rejected_when_robot_pose_is_outside_map(self) -> None:
        spec = GridSpec(width=10, height=10, resolution=0.1)
        data = [-1] * 100
        for y in range(6, 9):
            for x in range(6, 9):
                data[y * spec.width + x] = 0

        reachable = reachable_free_cell_indices(
            data,
            spec,
            robot_x=0.05,
            robot_y=0.05,
            maximum_seed_distance=0.5,
        )
        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.05,
            robot_y=0.05,
            min_cells=2,
            min_distance=0.0,
            max_distance=10.0,
            maximum_robot_free_seed_distance=0.5,
        )

        self.assertEqual(reachable, set())
        self.assertEqual(candidates, [])

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

    def test_physical_mapping_stage_leaves_meaningful_travel_after_tolerance(self) -> None:
        spec = GridSpec(width=45, height=5, resolution=0.05)
        data = [100] * (spec.width * spec.height)
        for x in range(1, 44):
            data[2 * spec.width + x] = 0
        data[2 * spec.width + 44] = -1

        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.125,
            robot_y=0.125,
            min_cells=1,
            min_distance=0.45,
            max_distance=7.0,
            goal_standoff=0.35,
            maximum_goal_step_distance=0.75,
        )

        self.assertEqual(len(candidates), 1)
        self.assertGreaterEqual(candidates[0].distance, 0.70)
        # Mapping overrides Nav2's xy tolerance to 0.15 m. The staged goal
        # must still demand substantially more than a tiny startup movement.
        self.assertGreater(candidates[0].distance - 0.15, 0.50)

    def test_frontier_beyond_goal_limit_uses_bounded_staged_goal(self) -> None:
        spec = GridSpec(width=40, height=5, resolution=0.1)
        data = [-1] * (spec.width * spec.height)
        for x in range(spec.width):
            data[1 * spec.width + x] = 100
            data[3 * spec.width + x] = 100
        data[2 * spec.width] = 100
        for x in range(1, 36):
            data[2 * spec.width + x] = 0

        candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.15,
            robot_y=0.25,
            min_cells=1,
            min_distance=0.0,
            max_distance=1.5,
            maximum_goal_step_distance=1.0,
        )

        self.assertEqual(len(candidates), 1)
        self.assertLessEqual(candidates[0].distance, 1.01)
        self.assertGreater(candidates[0].grid_x, 1)

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

    def test_narrow_connection_does_not_make_frontier_reachable(self) -> None:
        spec = GridSpec(width=20, height=9, resolution=0.1)
        data = [100] * (spec.width * spec.height)
        for y in range(2, 7):
            for x in range(1, 6):
                data[y * spec.width + x] = 0
            for x in range(14, 19):
                data[y * spec.width + x] = 0
        for x in range(6, 14):
            data[4 * spec.width + x] = 0
        for y in range(2, 7):
            data[y * spec.width + 19] = -1

        raw_candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.35,
            robot_y=0.45,
            min_cells=2,
            min_distance=0.0,
            max_distance=10.0,
        )
        safe_candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.35,
            robot_y=0.45,
            min_cells=2,
            min_distance=0.0,
            max_distance=10.0,
            minimum_obstacle_clearance=0.15,
        )
        wide_path_candidates = frontier_candidates(
            data,
            spec,
            robot_x=0.35,
            robot_y=0.45,
            min_cells=2,
            min_distance=0.0,
            max_distance=10.0,
            minimum_obstacle_clearance=0.0,
            minimum_path_obstacle_clearance=0.15,
        )

        self.assertEqual(len(raw_candidates), 1)
        self.assertEqual(safe_candidates, [])
        self.assertEqual(wide_path_candidates, [])

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

    def test_blacklist_uses_frontier_location_not_shared_staged_goal(self) -> None:
        spec = GridSpec(width=11, height=11, resolution=0.1)
        data = [-1] * (spec.width * spec.height)
        # A corridor has two distant gray boundaries, but their first staged
        # goals are close enough that a staged-goal blacklist would hide both.
        for coordinate in range(2, 9):
            data[5 * spec.width + coordinate] = 0
            data[4 * spec.width + coordinate] = 100
            data[6 * spec.width + coordinate] = 100

        initial = frontier_candidates(
            data,
            spec,
            robot_x=0.55,
            robot_y=0.55,
            min_cells=1,
            min_distance=0.0,
            max_distance=10.0,
            maximum_goal_step_distance=0.2,
        )
        self.assertEqual(len(initial), 2)

        alternatives = frontier_candidates(
            data,
            spec,
            robot_x=0.55,
            robot_y=0.55,
            min_cells=1,
            min_distance=0.0,
            max_distance=10.0,
            maximum_goal_step_distance=0.2,
            blacklisted=[(initial[0].frontier_x, initial[0].frontier_y)],
            blacklist_radius=0.25,
        )

        self.assertEqual(len(alternatives), 1)

        blocked_route = frontier_candidates(
            data,
            spec,
            robot_x=0.55,
            robot_y=0.55,
            min_cells=1,
            min_distance=0.0,
            max_distance=10.0,
            maximum_goal_step_distance=0.2,
            staged_blacklisted=[(initial[0].x, initial[0].y)],
            staged_blacklist_radius=0.15,
        )
        self.assertEqual(len(blocked_route), 1)
        self.assertGreater(
            math.hypot(
                blocked_route[0].x - initial[0].x,
                blocked_route[0].y - initial[0].y,
            ),
            0.15,
        )

    def test_invalid_grid_size_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            frontier_cell_indices([0], GridSpec(2, 2, 0.1))

    def test_distance_weight_stays_normal_below_the_stuck_threshold(self) -> None:
        weight = frontier_distance_weight(
            consecutive_goal_failures=5,
            stuck_failure_threshold=6,
            normal_distance_weight=0.45,
            stuck_distance_weight=0.15,
        )

        self.assertEqual(weight, 0.45)

    def test_distance_weight_relaxes_at_the_stuck_threshold(self) -> None:
        weight = frontier_distance_weight(
            consecutive_goal_failures=6,
            stuck_failure_threshold=6,
            normal_distance_weight=0.45,
            stuck_distance_weight=0.15,
        )

        self.assertEqual(weight, 0.15)

    def test_distance_weight_relief_can_be_disabled(self) -> None:
        weight = frontier_distance_weight(
            consecutive_goal_failures=50,
            stuck_failure_threshold=0,
            normal_distance_weight=0.45,
            stuck_distance_weight=0.15,
        )

        self.assertEqual(weight, 0.45)

    def test_goal_step_distance_stays_normal_below_the_stuck_threshold(self) -> None:
        step = frontier_goal_step_distance(
            consecutive_goal_failures=5,
            stuck_failure_threshold=6,
            normal_step_distance=1.0,
            stuck_step_distance=0.35,
        )

        self.assertEqual(step, 1.0)

    def test_goal_step_distance_shrinks_at_the_stuck_threshold(self) -> None:
        step = frontier_goal_step_distance(
            consecutive_goal_failures=6,
            stuck_failure_threshold=6,
            normal_step_distance=1.0,
            stuck_step_distance=0.35,
        )

        self.assertEqual(step, 0.35)

    def test_goal_step_distance_relief_can_be_disabled(self) -> None:
        step = frontier_goal_step_distance(
            consecutive_goal_failures=50,
            stuck_failure_threshold=0,
            normal_step_distance=1.0,
            stuck_step_distance=0.35,
        )

        self.assertEqual(step, 1.0)

    def test_stuck_goal_step_distance_produces_a_shorter_staged_goal(self) -> None:
        # Same corridor as test_far_frontier_is_staged_along_connected_free_path,
        # just scored with the shrunken stuck-relief step instead of the
        # normal one. The staged goal must land closer to the robot.
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
            min_distance=0.15,
            max_distance=7.0,
            goal_standoff=0.1,
            maximum_goal_step_distance=0.35,
        )

        self.assertTrue(candidates)
        self.assertLessEqual(candidates[0].distance, 0.36)

    def test_distance_weight_parameter_reaches_frontier_scoring(self) -> None:
        # Reuses the two-cluster map from
        # test_far_frontier_is_staged_along_connected_free_path: a near
        # cluster and a longer, larger one further off. At the normal weight
        # distance dominates and the near cluster wins; a relaxed weight
        # (as frontier_distance_weight returns once stuck) must let frontier
        # size compete, changing which candidate sorts first.
        width, height = 20, 6
        data = [-1] * (width * height)

        def set_cell(x: int, y: int, value: int) -> None:
            data[y * width + x] = value

        for x in range(width):
            for y in range(height):
                set_cell(x, y, 0)
        for x in range(3, 6):
            set_cell(x, 2, -1)
        for x in range(14, 20):
            for y in range(1, 5):
                set_cell(x, y, -1)
        spec = GridSpec(width=width, height=height, resolution=0.1)

        near_first = frontier_candidates(
            data, spec, robot_x=0.15, robot_y=0.25, min_cells=1, distance_weight=5.0
        )
        far_first = frontier_candidates(
            data, spec, robot_x=0.15, robot_y=0.25, min_cells=1, distance_weight=0.0
        )

        self.assertTrue(near_first)
        self.assertTrue(far_first)
        self.assertNotEqual(
            (near_first[0].frontier_x, near_first[0].frontier_y),
            (far_first[0].frontier_x, far_first[0].frontier_y),
        )

    def test_unknown_region_size_counts_connected_unknown_cells(self) -> None:
        width, height = 10, 5
        data = [0] * (width * height)
        data[2 * width + 8] = -1
        spec = GridSpec(width=width, height=height, resolution=0.1)

        size = unknown_region_size(data, spec, [2 * width + 7])

        self.assertEqual(size, 1)

    def test_unknown_region_size_grows_with_a_larger_pocket(self) -> None:
        width, height = 10, 6
        data = [0] * (width * height)
        for y in range(0, 4):
            for x in range(7, 10):
                data[y * width + x] = -1
        spec = GridSpec(width=width, height=height, resolution=0.1)

        size = unknown_region_size(data, spec, [2 * width + 6])

        self.assertEqual(size, 12)

    def test_unknown_region_size_respects_the_cap(self) -> None:
        width, height = 40, 40
        data = [-1] * (width * height)
        data[width * (height // 2) + 1] = 0
        spec = GridSpec(width=width, height=height, resolution=0.1)

        size = unknown_region_size(
            data, spec, [width * (height // 2) + 1], cap=50
        )

        self.assertEqual(size, 50)

    def test_unknown_gain_weight_prefers_the_larger_unmapped_sector(self) -> None:
        # Two doorways off one small room, equidistant from the robot: a
        # narrow gap into a big unmapped area on the left, a similar gap
        # into a tiny pocket on the right. Frontier length and distance are
        # close enough that a real unknown_gain_weight should decide it.
        width, height = 24, 9
        wall = 100
        data = [wall] * (width * height)

        def set_cell(x: int, y: int, value: int) -> None:
            data[y * width + x] = value

        for x in range(10, 14):
            for y in range(3, 6):
                set_cell(x, y, 0)
        set_cell(9, 4, 0)
        set_cell(14, 4, 0)
        for x in range(1, 9):
            for y in range(1, 8):
                set_cell(x, y, -1)
        for x in range(15, 17):
            for y in range(3, 6):
                set_cell(x, y, -1)
        spec = GridSpec(width=width, height=height, resolution=0.2)

        unweighted = frontier_candidates(
            data,
            spec,
            robot_x=2.3,
            robot_y=0.9,
            min_cells=1,
            distance_weight=0.1,
            unknown_gain_weight=0.0,
        )
        weighted = frontier_candidates(
            data,
            spec,
            robot_x=2.3,
            robot_y=0.9,
            min_cells=1,
            distance_weight=0.1,
            unknown_gain_weight=5.0,
        )

        self.assertGreaterEqual(len(unweighted), 2)
        self.assertGreaterEqual(len(weighted), 2)
        # The doorway toward the big left room (lower grid_x) must win once
        # unknown area counts, regardless of which side the plain boundary
        # -length/-distance score preferred.
        self.assertLess(weighted[0].grid_x, 12)


if __name__ == "__main__":
    unittest.main()
