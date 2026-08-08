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
