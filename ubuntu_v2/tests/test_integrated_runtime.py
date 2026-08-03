from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from robot_project.main import _arguments  # noqa: E402
from robot_project.runtime import components_for_mode  # noqa: E402


class IntegratedRuntimeTests(unittest.TestCase):
    def test_perception_mode_runs_detection_only(self) -> None:
        self.assertEqual(components_for_mode("perception"), ("perception",))

    def test_follow_mode_includes_perception(self) -> None:
        self.assertEqual(
            components_for_mode("follow"), ("perception", "follow")
        )

    def test_navigation_is_mutually_exclusive_with_follow(self) -> None:
        self.assertEqual(components_for_mode("navigation"), ("navigation",))

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            components_for_mode("everything")

    def test_ros_arguments_are_preserved(self) -> None:
        parsed, ros_args = _arguments(
            ["--mode", "follow", "--ros-args", "-p", "linear_speed:=0.3"]
        )
        self.assertEqual(parsed.mode, "follow")
        self.assertEqual(
            ros_args, ["--ros-args", "-p", "linear_speed:=0.3"]
        )


if __name__ == "__main__":
    unittest.main()
