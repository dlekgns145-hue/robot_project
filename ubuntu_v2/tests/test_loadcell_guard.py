from __future__ import annotations

import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch


def load_loadcell_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_qos = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, depth=10):
            self.depth = depth
            self.reliability = None
            self.durability = None

    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    std_msgs_msg.Float32 = type("Float32", (), {})
    std_msgs_msg.String = type("String", (), {})
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = type("Trigger", (), {})
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }
    path = (
        Path(__file__).resolve().parents[2]
        / "robot_docker"
        / "loadcell_guard.py"
    )
    spec = importlib.util.spec_from_file_location("loadcell_guard_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class LoadcellGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.guard = load_loadcell_module()

    def test_raw_to_grams_applies_tare_and_scale(self) -> None:
        grams = self.guard.raw_to_grams(raw=1_200_000, tare_offset=1_000_000, grams_per_count=0.001)

        self.assertAlmostEqual(grams, 200.0)

    def test_raw_to_grams_rejects_uncalibrated_scale(self) -> None:
        with self.assertRaises(ValueError):
            self.guard.raw_to_grams(raw=1_200_000, tare_offset=1_000_000, grams_per_count=0.0)

    def test_classify_weight_reports_unavailable_without_a_reading(self) -> None:
        state = self.guard.classify_weight(None, low_threshold_g=100.0, high_threshold_g=5000.0)

        self.assertEqual(state, "unavailable")

    def test_classify_weight_flags_below_threshold(self) -> None:
        state = self.guard.classify_weight(50.0, low_threshold_g=100.0, high_threshold_g=5000.0)

        self.assertEqual(state, "below")

    def test_classify_weight_flags_above_threshold(self) -> None:
        state = self.guard.classify_weight(6000.0, low_threshold_g=100.0, high_threshold_g=5000.0)

        self.assertEqual(state, "above")

    def test_classify_weight_reports_normal_inside_the_band(self) -> None:
        state = self.guard.classify_weight(2500.0, low_threshold_g=100.0, high_threshold_g=5000.0)

        self.assertEqual(state, "normal")

    def test_classify_weight_band_edges_are_inclusive(self) -> None:
        self.assertEqual(
            self.guard.classify_weight(100.0, low_threshold_g=100.0, high_threshold_g=5000.0),
            "normal",
        )
        self.assertEqual(
            self.guard.classify_weight(5000.0, low_threshold_g=100.0, high_threshold_g=5000.0),
            "normal",
        )


class LoadcellDeploymentWiringTests(unittest.TestCase):
    """Guards the exact class of bug HANDOFF.md 6절 documents repeatedly:
    a node's launch/code changes without its Dockerfile COPY / entrypoint /
    compose entries keeping up, so the container silently lacks the file or
    never starts the mode at all."""

    def test_loadcell_guard_is_copied_into_the_image(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        dockerfile_text = (docker_dir / "Dockerfile").read_text()

        self.assertIn("loadcell_guard.py", dockerfile_text)
        self.assertIn("gpiozero", dockerfile_text)
        self.assertIn("lgpio", dockerfile_text)

    def test_entrypoint_has_a_loadcell_mode(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        entrypoint_text = (docker_dir / "entrypoint.sh").read_text()

        self.assertIn("loadcell)", entrypoint_text)
        self.assertIn("loadcell_guard.py", entrypoint_text)

    def test_compose_defines_a_standalone_loadcell_service(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        compose_text = (docker_dir / "compose.yaml").read_text()

        self.assertIn("loadcell:", compose_text)
        self.assertIn('command: ["loadcell"]', compose_text)
        # Must not be gated behind a profile: weight thresholding has to work
        # standalone on the robot, same as camera-safety.
        loadcell_block = compose_text.split("loadcell:", 1)[1].split("\n\n", 1)[0]
        self.assertNotIn("profiles:", loadcell_block)

    def test_bridge_exposes_loadcell_status_over_the_socket(self) -> None:
        docker_dir = Path(__file__).resolve().parents[2] / "robot_docker"
        bridge_text = (docker_dir / "robot_cmd_bridge.py").read_text()

        self.assertIn("/loadcell_guard/status", bridge_text)
        self.assertIn('response["loadcell"]', bridge_text)


if __name__ == "__main__":
    unittest.main()
