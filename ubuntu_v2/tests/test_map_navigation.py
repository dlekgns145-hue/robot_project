from __future__ import annotations

import math
import base64
import sys
import unittest
import zlib
from pathlib import Path


GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
sys.path.insert(0, str(GUI_DIR))

from map_navigation import (  # noqa: E402
    MapMetadata,
    decode_map_image,
    pixel_to_world,
    world_to_pixel,
)


class MapCoordinateTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
