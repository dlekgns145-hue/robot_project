from __future__ import annotations

import sys
import unittest
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
sys.path.insert(0, str(GUI_DIR))

from control_logic import FollowSettings, movement, servo_target, smooth_servo  # noqa: E402


class ControlLogicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = FollowSettings()

    def test_center_target_keeps_camera_centered(self) -> None:
        self.assertEqual(servo_target(160, 320, self.settings), 0)

    def test_servo_movement_is_step_limited(self) -> None:
        self.assertEqual(smooth_servo(0, 60, 6), 6)
        self.assertEqual(smooth_servo(0, -60, 6), -6)

    def test_robot_aligns_before_moving_forward(self) -> None:
        linear, angular, mode = movement(30, 40, 240, self.settings)
        self.assertEqual(linear, 0.0)
        self.assertNotEqual(angular, 0.0)
        self.assertEqual(mode, "ALIGN")

    def test_robot_stops_when_target_is_close(self) -> None:
        linear, angular, mode = movement(0, 140, 240, self.settings)
        self.assertEqual((linear, angular, mode), (0.0, 0.0, "STOP"))


if __name__ == "__main__":
    unittest.main()
