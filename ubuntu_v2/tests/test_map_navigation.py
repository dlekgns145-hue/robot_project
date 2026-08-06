from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
sys.path.insert(0, str(GUI_DIR))

from map_navigation import MapMetadata, pixel_to_world, world_to_pixel  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
