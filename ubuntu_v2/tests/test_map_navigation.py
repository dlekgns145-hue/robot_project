from __future__ import annotations

import math
import base64
import os
import sys
import unittest
import zlib
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
sys.path.insert(0, str(GUI_DIR))

from map_navigation import (  # noqa: E402
    MapMetadata,
    decode_map_image,
    pixel_to_world,
    world_to_pixel,
)
from PySide6.QtWidgets import QApplication  # noqa: E402


class MapCoordinateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_pixel_world_round_trip_for_saved_map_orientation(self) -> None:
        metadata = MapMetadata(180, 193, 0.05, -5.05, -1.52)

        world = pixel_to_world(72.25, 100.75, metadata)
        pixel = world_to_pixel(*world, metadata)

        self.assertAlmostEqual(pixel[0], 72.25)
        self.assertAlmostEqual(pixel[1], 100.75)

    def test_origin_yaw_is_applied_in_both_directions(self) -> None:
        metadata = MapMetadata(20, 30, 0.1, 2.0, -1.0, math.pi / 2.0)

        world = pixel_to_world(4.0, 8.0, metadata)
        pixel = world_to_pixel(*world, metadata)

        self.assertAlmostEqual(pixel[0], 4.0)
        self.assertAlmostEqual(pixel[1], 8.0)

    def test_compressed_map_transfer_is_decoded(self) -> None:
        pgm = b"P5\n2 2\n255\n\x00\xff\xff\x00"
        payload = {
            "image_base64": base64.b64encode(zlib.compress(pgm)).decode("ascii"),
            "image_encoding": "zlib+base64",
        }

        self.assertEqual(decode_map_image(payload), pgm)

    def test_camera_layer_visibility_and_opacity_are_bounded(self) -> None:
        from map_navigation import NavigationMapView

        view = NavigationMapView()
        view.set_texture_visible(False)
        view.set_texture_opacity(1.5)

        self.assertFalse(view.texture_visible)
        self.assertEqual(view.texture_opacity, 1.0)

        view.set_texture_opacity(-0.2)
        self.assertEqual(view.texture_opacity, 0.0)

    def test_visual_obstacle_layer_is_decoded_separately(self) -> None:
        from map_navigation import NavigationMapView

        occupancy = np.array([[254, 0], [205, 254]], dtype=np.uint8)
        obstacle = np.zeros((2, 2, 4), dtype=np.uint8)
        obstacle[0, 1] = (20, 140, 35, 225)
        _ok, pgm = cv2.imencode(".pgm", occupancy)
        _ok, png = cv2.imencode(".png", obstacle)
        view = NavigationMapView()

        view.set_map_payload(
            {
                "image_base64": base64.b64encode(pgm).decode("ascii"),
                "obstacle_texture_base64": base64.b64encode(png).decode("ascii"),
                "width": 2,
                "height": 2,
                "resolution": 0.05,
                "origin_x": 0.0,
                "origin_y": 0.0,
            }
        )

        self.assertTrue(view.has_texture)
        self.assertTrue(view.has_obstacle_texture)


if __name__ == "__main__":
    unittest.main()
