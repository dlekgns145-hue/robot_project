from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


DOCKER_DIR = Path(__file__).resolve().parents[2] / "robot_docker"
sys.path.insert(0, str(DOCKER_DIR))

from map_texture_core import SavedMapInfo  # noqa: E402
from obstacle_texture_fusion import (  # noqa: E402
    CameraLidarCalibration,
    ObstacleObservation,
    ObstacleTextureAccumulator,
    observations_from_scan,
    occupancy_grid_to_pgm,
    render_obstacle_texture,
)


class ObstacleTextureFusionTests(unittest.TestCase):
    def test_scan_endpoint_uses_matching_camera_direction_and_robot_pose(self) -> None:
        frame = np.full((100, 160, 3), (25, 90, 210), dtype=np.uint8)
        calibration = CameraLidarCalibration(
            horizontal_fov_deg=90.0,
            vertical_fov_deg=60.0,
            camera_pitch_down_deg=0.0,
            camera_height_m=0.2,
            lidar_x_offset_m=0.0,
            sample_height_fraction=0.2,
            scan_stride=1,
        )

        observations = observations_from_scan(
            frame,
            [float("inf"), 2.0, float("inf")],
            angle_min=-0.2,
            angle_increment=0.2,
            range_min=0.1,
            range_max=8.0,
            robot_x=1.0,
            robot_y=2.0,
            robot_yaw=math.pi / 2.0,
            calibration=calibration,
        )

        self.assertEqual(len(observations), 1)
        self.assertAlmostEqual(observations[0].x, 1.0, places=4)
        self.assertAlmostEqual(observations[0].y, 4.0, places=4)
        self.assertEqual(observations[0].bgr, (25, 90, 210))
        self.assertGreater(observations[0].weight, 0.1)

    def test_accumulator_averages_repeated_world_cell_colors(self) -> None:
        accumulator = ObstacleTextureAccumulator(cell_size_m=0.1, maximum_cells=2)
        accumulator.add(
            [
                ObstacleObservation(1.01, 2.01, (10, 20, 30), 1.0),
                ObstacleObservation(1.02, 2.02, (30, 40, 50), 1.0),
            ]
        )

        observations = accumulator.observations()

        self.assertEqual(len(observations), 1)
        self.assertEqual(observations[0].bgr, (20, 30, 40))
        self.assertEqual(observations[0].hits, 2)

    def test_render_paints_only_lidar_occupied_cells(self) -> None:
        info = SavedMapInfo(60, 60, 0.1, 0.0, 0.0, 0.0)
        occupancy = np.full((60, 60), 254, dtype=np.uint8)
        occupancy[39:42, 19:22] = 0
        observation = ObstacleObservation(2.0, 2.0, (40, 120, 220), 1.0, hits=3)

        base, layer, metadata = render_obstacle_texture(
            occupancy,
            info,
            [observation],
            endpoint_tolerance_pixels=2,
            paint_radius_pixels=1,
        )

        self.assertEqual(base.shape, (60, 60, 3))
        observed = layer[..., 3] > 0
        self.assertGreater(int(np.count_nonzero(observed)), 0)
        self.assertTrue(np.all(occupancy[observed] < 65))
        self.assertEqual(metadata["observation_cells"], 1)
        self.assertGreater(metadata["total_hits"], 0)

    def test_occupancy_grid_conversion_flips_ros_row_order(self) -> None:
        pgm = occupancy_grid_to_pgm(
            [100, 0, -1, 0, 100, -1],
            width=3,
            height=2,
        )

        np.testing.assert_array_equal(
            pgm,
            np.asarray([[254, 0, 205], [0, 254, 205]], dtype=np.uint8),
        )


if __name__ == "__main__":
    unittest.main()
