from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


DOCKER_DIR = Path(__file__).resolve().parents[2] / "robot_docker"
sys.path.insert(0, str(DOCKER_DIR))

from map_texture_core import (  # noqa: E402
    CameraSample,
    SavedMapInfo,
    compose_texture,
    compose_visual_layers,
    load_saved_map,
    world_to_map_pixel,
)
from calibrate_map_texture import (  # noqa: E402
    build_calibration_preview,
    normalize_map_output,
)


class MapTextureCoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.info = SavedMapInfo(
            width=100,
            height=100,
            resolution=0.1,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
        )

    @staticmethod
    def _sample(color: tuple[int, int, int] = (20, 80, 220)) -> CameraSample:
        frame = np.full((80, 120, 3), color, dtype=np.uint8)
        ok, encoded = cv2.imencode(".jpg", frame)
        if not ok:
            raise AssertionError("test JPEG could not be encoded")
        return CameraSample(encoded.tobytes(), x=2.0, y=2.0, yaw=0.0)

    def test_world_to_pixel_applies_rotated_map_origin(self) -> None:
        rotated = SavedMapInfo(20, 30, 0.1, 2.0, -1.0, math.pi / 2.0)

        pixel = world_to_map_pixel(1.45, -0.55, rotated)

        self.assertAlmostEqual(pixel[0], 4.0)
        self.assertAlmostEqual(pixel[1], 24.0)

    def test_camera_pixels_are_painted_only_on_known_free_cells(self) -> None:
        occupancy = np.full((100, 100), 254, dtype=np.uint8)
        occupancy[75, 25] = 0
        occupancy[76, 25] = 205

        texture = compose_texture(
            occupancy,
            self.info,
            [self._sample()],
            near_m=0.2,
            far_m=1.5,
            near_width_m=0.8,
            far_width_m=1.6,
            source_top_fraction=0.45,
        )

        colored = np.any(texture != texture[..., :1], axis=2)
        self.assertGreater(int(np.count_nonzero(colored)), 20)
        np.testing.assert_array_equal(texture[75, 25], [0, 0, 0])
        np.testing.assert_array_equal(texture[76, 25], [205, 205, 205])

    def test_invalid_projection_and_map_dimensions_are_rejected(self) -> None:
        occupancy = np.full((100, 100), 254, dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, "greater than"):
            compose_texture(
                occupancy,
                self.info,
                [self._sample()],
                near_m=1.0,
                far_m=0.5,
            )
        with self.assertRaisesRegex(ValueError, "dimensions"):
            compose_texture(occupancy[:50], self.info, [])

    def test_obstacle_appearance_and_material_are_separate_from_occupancy(self) -> None:
        occupancy = np.full((100, 100), 254, dtype=np.uint8)
        occupancy[70:82, 23:29] = 0

        texture, obstacle_layer, materials = compose_visual_layers(
            occupancy,
            self.info,
            [self._sample((25, 150, 35))],
            near_m=0.2,
            far_m=1.5,
            near_width_m=0.8,
            far_width_m=1.6,
            source_top_fraction=0.45,
        )

        obstacle_pixels = obstacle_layer[..., 3] > 0
        self.assertGreater(int(np.count_nonzero(obstacle_pixels)), 10)
        np.testing.assert_array_equal(texture[75, 25], [0, 0, 0])
        self.assertEqual(materials["dominant"], "vegetation")
        self.assertGreater(materials["counts"]["vegetation"], 10)

    def test_saved_map_metadata_is_loaded_and_validated(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "orchard_map"
            cv2.imwrite(str(output) + ".pgm", np.full((3, 4), 254, np.uint8))
            (Path(str(output) + ".yaml")).write_text(
                "image: orchard_map.pgm\n"
                "resolution: 0.05\n"
                "origin: [-1.0, 2.0, 0.25]\n",
                encoding="utf-8",
            )

            image, info = load_saved_map(str(output))

            self.assertEqual(image.shape, (3, 4))
            self.assertEqual(info.width, 4)
            self.assertEqual(info.height, 3)
            self.assertAlmostEqual(info.origin_yaw, 0.25)

    def test_offline_calibration_preview_uses_live_projection_core(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            map_output = root / "orchard_map"
            occupancy = np.full((100, 100), 254, np.uint8)
            cv2.imwrite(str(map_output) + ".pgm", occupancy)
            Path(str(map_output) + ".yaml").write_text(
                "resolution: 0.1\norigin: [0.0, 0.0, 0.0]\n",
                encoding="utf-8",
            )
            image_path = root / "camera.jpg"
            cv2.imwrite(str(image_path), np.full((80, 120, 3), (20, 80, 220), np.uint8))
            preview_path = root / "preview.png"

            projected = build_calibration_preview(
                map_output=str(map_output) + ".yaml",
                image_path=str(image_path),
                output_path=str(preview_path),
                x=2.0,
                y=2.0,
                yaw=0.0,
                near_m=0.2,
                far_m=1.5,
                near_width_m=0.8,
                far_width_m=1.6,
                source_top_fraction=0.45,
            )

            self.assertGreater(projected, 20)
            self.assertTrue(preview_path.is_file())
            self.assertEqual(normalize_map_output(str(map_output) + ".pgm"), str(map_output))


if __name__ == "__main__":
    unittest.main()
