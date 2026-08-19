from __future__ import annotations

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np


def load_camera_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    path = (
        Path(__file__).resolve().parents[2]
        / "robot_docker"
        / "camera_obstacle_guard.py"
    )
    spec = importlib.util.spec_from_file_location("camera_guard_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class CameraObstacleGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.guard = load_camera_module()

    def test_uniform_floor_produces_no_obstacle(self) -> None:
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertEqual(components, 0)
        self.assertTrue(all(math.isinf(value) for value in ranges))

    def test_large_foreground_object_marks_central_rays(self) -> None:
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (135, 135), (185, 205), (20, 20, 210), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertGreaterEqual(components, 1)
        central = ranges[len(ranges) // 2 - 3:len(ranges) // 2 + 4]
        self.assertTrue(any(math.isfinite(value) for value in central))
        self.assertLess(min(value for value in central if math.isfinite(value)), 1.0)

    def test_bright_achromatic_floor_reflection_is_not_an_obstacle(self) -> None:
        # A glossy floor mirroring an overhead light reads as a near-white,
        # low-chroma patch -- bright and different enough from the matte
        # floor reference to pass the raw threshold, but physically flat.
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (135, 135), (185, 205), (235, 235, 235), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertEqual(components, 0)
        self.assertTrue(all(math.isinf(value) for value in ranges))

    def test_bright_colored_object_still_counts_despite_high_luminance(self) -> None:
        # Guards against the reflection filter over-triggering: a light but
        # clearly coloured (high-chroma) object must still be treated as a
        # real obstacle, not swept up as a "reflection".
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (135, 135), (185, 205), (40, 200, 235), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertGreaterEqual(components, 1)

    def test_wide_dark_achromatic_band_is_treated_as_shadow(self) -> None:
        # A hard-edged shadow (not speckled, so it clears the fill-ratio
        # gate) spanning most of the floor width -- near-black and
        # essentially colourless, same signature seen on the real robot.
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (16, 140), (276, 195), (20, 20, 20), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertEqual(components, 0)
        self.assertTrue(all(math.isinf(value) for value in ranges))

    def test_narrow_dark_achromatic_object_still_counts(self) -> None:
        # Same colour/darkness as the shadow above, but narrow -- a real
        # dark object (cable, chair leg) must not get swept up by the
        # shadow filter just for being colourless.
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (135, 140), (185, 195), (20, 20, 20), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertGreaterEqual(components, 1)

    def test_sparse_hollow_pattern_is_not_an_obstacle(self) -> None:
        # A shadow or floor-reflection patch is fragmented within its
        # bounding box, unlike a real object's mostly-solid silhouette.
        # A wide, thin-outline shape is a convenient stand-in: same colour
        # and size as a real object, but a low fill ratio.
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (100, 100), (220, 200), (20, 20, 210), 6)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertEqual(components, 0)
        self.assertTrue(all(math.isinf(value) for value in ranges))

    def test_debug_sink_records_why_each_candidate_was_or_was_not_flagged(self) -> None:
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (135, 135), (185, 205), (20, 20, 210), -1)
        debug_sink: list[dict] = []

        self.guard.obstacle_ranges_from_frame(frame, debug_sink=debug_sink)

        self.assertTrue(debug_sink)
        entry = debug_sink[0]
        self.assertEqual(entry["status"], "accepted")
        for key in ("bbox", "area", "mean_luminance", "mean_chroma", "distance_m"):
            self.assertIn(key, entry)

    def test_two_centimeter_floor_lip_is_treated_as_traversable(self) -> None:
        frame = np.full((240, 320, 3), (90, 110, 100), dtype=np.uint8)
        cv2.rectangle(frame, (80, 205), (240, 216), (20, 20, 210), -1)

        ranges, components = self.guard.obstacle_ranges_from_frame(frame)

        self.assertEqual(components, 0)
        self.assertTrue(all(math.isinf(value) for value in ranges))

    def test_height_estimate_blocks_when_geometry_is_invalid(self) -> None:
        estimated = self.guard.estimate_component_height_m(
            image_height=240,
            top_y=200,
            contact_y=180,
        )

        self.assertTrue(math.isinf(estimated))

    def test_single_frame_camera_noise_is_not_confirmed(self) -> None:
        history = [
            [math.inf, 0.8, math.inf],
            [math.inf, math.inf, math.inf],
            [math.inf, 0.9, math.inf],
        ]

        confirmed = self.guard.temporally_confirm_ranges(history, minimum_hits=2)

        self.assertTrue(math.isinf(confirmed[0]))
        self.assertAlmostEqual(confirmed[1], 0.85)


if __name__ == "__main__":
    unittest.main()
