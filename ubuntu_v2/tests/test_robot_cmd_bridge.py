from __future__ import annotations

import base64
import importlib.util
import json
import os
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch


class FakeNode:
    pass


class Twist:
    def __init__(self) -> None:
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class String:
    def __init__(self) -> None:
        self.data = ""


class Trigger:
    class Request:
        pass


class RecordingPublisher:
    def __init__(self) -> None:
        self.messages: list[Twist] = []

    def publish(self, message: Twist) -> None:
        self.messages.append(message)


def load_bridge_module():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_action = types.ModuleType("rclpy.action")
    rclpy_action.ActionClient = type("ActionClient", (), {})
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=1)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    rclpy_qos.QoSProfile = type("QoSProfile", (), {})
    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = types.SimpleNamespace(
        STATUS_SUCCEEDED=4, STATUS_CANCELED=5
    )
    nav2_msgs = types.ModuleType("nav2_msgs")
    nav2_msgs_action = types.ModuleType("nav2_msgs.action")
    nav2_msgs_action.NavigateToPose = type("NavigateToPose", (), {})
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = Twist
    geometry_msgs_msg.PoseWithCovarianceStamped = type(
        "PoseWithCovarianceStamped", (), {}
    )
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    std_msgs_msg.Int32 = type("Int32", (), {})
    std_msgs_msg.String = String
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = type("SetBool", (), {})
    std_srvs_srv.Trigger = Trigger
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.action": rclpy_action,
        "rclpy.qos": rclpy_qos,
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "nav2_msgs": nav2_msgs,
        "nav2_msgs.action": nav2_msgs_action,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }
    path = Path(__file__).resolve().parents[2] / "robot_docker" / "robot_cmd_bridge.py"
    spec = importlib.util.spec_from_file_location("robot_cmd_bridge_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


class CommandLeaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.bridge = load_bridge_module()

    def make_node(self):
        node = object.__new__(self.bridge.CmdBridgeNode)
        node.lock = threading.Lock()
        node.last_cmd_time = 0.0
        node._timeout_stop_published = False
        node.navigation_mode = False
        node.avoid_state = "NORMAL"
        node.pub = RecordingPublisher()
        node.server_linear = 0.0
        node.server_angular = 0.0
        node.last_server_cmd_at = 0.0
        node.last_navigation_lease_at = 0.0
        node.navigation_lease_active = False
        node._last_scan_at = time.monotonic()
        node.front_min_dist = 10.0
        node.rear_min_dist = 10.0
        node._last_camera_scan_at = 0.0
        node.camera_front_min_dist = float("inf")
        return node

    def test_stale_command_publishes_one_stop_then_releases_topic(self) -> None:
        node = self.make_node()

        node.control_loop()
        node.control_loop()

        self.assertEqual(len(node.pub.messages), 1)
        self.assertEqual(node.pub.messages[0].linear.x, 0.0)
        self.assertEqual(node.pub.messages[0].angular.z, 0.0)

    def test_new_command_rearms_the_one_shot_timeout_stop(self) -> None:
        node = self.make_node()
        node.desired_linear = 0.0
        node.desired_angular = 0.0
        node._last_servo_pan_sent = None

        node.control_loop()
        node.publish_cmd(0.2, 0.0)
        node.last_cmd_time = 0.0
        node.control_loop()

        self.assertEqual(len(node.pub.messages), 2)

    def test_navigation_mode_rejects_gui_motor_commands(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.desired_linear = 0.0
        node.desired_angular = 0.0
        node._last_servo_pan_sent = None

        node.publish_cmd(0.2, 0.1)
        node.control_loop()

        self.assertEqual(node.desired_linear, 0.0)
        self.assertEqual(node.desired_angular, 0.0)
        self.assertEqual(len(node.pub.messages), 1)
        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_fresh_server_command_passes_through_local_safety_gate(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        command = Twist()
        command.linear.x = 0.1
        command.angular.z = -0.1

        node.server_cmd_callback(command)
        node.control_loop()

        self.assertEqual(len(node.pub.messages), 1)
        self.assertAlmostEqual(node.pub.messages[0].linear.x, 0.1)
        self.assertAlmostEqual(node.pub.messages[0].angular.z, -0.1)

    def test_pure_rotation_is_boosted_over_motor_deadzone(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(0.0, 0.08)

        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.18)

    def test_small_driving_steering_noise_is_removed(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(0.08, -0.02)

        self.assertAlmostEqual(linear, 0.08)
        self.assertEqual(angular, 0.0)

    def test_server_velocity_is_clamped(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(1.0, -1.0)

        self.assertAlmostEqual(linear, 0.12)
        self.assertAlmostEqual(angular, -0.18)

    def test_navigation_lease_enables_and_disables_motor_ownership(self) -> None:
        node = self.make_node()
        message = types.SimpleNamespace(data=True)

        node.navigation_lease_callback(message)

        self.assertTrue(node.navigation_mode)
        self.assertTrue(node.navigation_lease_active)
        message.data = False
        node.navigation_lease_callback(message)
        self.assertFalse(node.navigation_mode)

    def test_stale_server_command_is_stopped_on_robot(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = 0.0

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_local_camera_obstacle_blocks_server_forward_motion(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.4

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_rear_lidar_obstacle_blocks_server_reverse_motion(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = -0.08
        node.last_server_cmd_at = time.monotonic()
        node.rear_min_dist = 0.3

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_heartbeat_does_not_become_a_motor_command(self) -> None:
        commands = []
        node = types.SimpleNamespace(
            publish_cmd=lambda *args: commands.append(args),
            navigation_snapshot=lambda: {"state": "idle", "active": False},
        )

        self.bridge.handle_socket_command(node, {"heartbeat": 1})

        self.assertEqual(commands, [])

    def test_emergency_stop_takes_priority_over_heartbeat_marker(self) -> None:
        commands = []
        node = types.SimpleNamespace(
            publish_cmd=lambda *args: commands.append(args),
            navigation_snapshot=lambda: {"state": "idle", "active": False},
        )

        self.bridge.handle_socket_command(
            node, {"heartbeat": 1, "emergency_stop": 1}
        )

        self.assertEqual(commands, [(0.0, 0.0, None, True)])

    def test_remote_navigation_command_is_forwarded_to_action_owner(self) -> None:
        goals = []
        node = types.SimpleNamespace(
            start_navigation=lambda x, y, yaw: goals.append((x, y, yaw)),
            navigation_snapshot=lambda: {"state": "sending", "active": True},
        )

        response = self.bridge.handle_socket_command(
            node, {"type": "navigate", "x": 1.2, "y": -0.4, "yaw": 0.5}
        )

        self.assertEqual(goals, [(1.2, -0.4, 0.5)])
        self.assertTrue(response["ok"])
        self.assertTrue(response["navigation"]["active"])

    def test_mapping_command_and_status_are_returned(self) -> None:
        commands = []
        node = types.SimpleNamespace(
            mapping_command=lambda command: commands.append(command) or "started",
            mapping_snapshot=lambda: {"state": "starting", "enabled": True},
            navigation_snapshot=lambda: {"state": "idle", "active": False},
        )

        response = self.bridge.handle_socket_command(
            node, {"type": "mapping_start"}
        )

        self.assertEqual(commands, ["mapping_start"])
        self.assertTrue(response["command_result"]["ok"])
        self.assertTrue(response["mapping"]["enabled"])

    def test_map_payload_includes_fresh_obstacle_material_layers(self) -> None:
        node = object.__new__(self.bridge.CmdBridgeNode)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image_path = root / "orchard_map.pgm"
            yaml_path = root / "orchard_map.yaml"
            texture_path = root / "orchard_map_texture.png"
            obstacle_path = root / "orchard_map_obstacles.png"
            materials_path = root / "orchard_map_materials.json"
            image_path.write_bytes(b"P2\n2 2\n255\n254 0\n205 254\n")
            yaml_path.write_text(
                "resolution: 0.05\norigin: [0.0, 0.0, 0.0]\n",
                encoding="utf-8",
            )
            texture_path.write_bytes(b"texture-png")
            obstacle_path.write_bytes(b"obstacle-png")
            materials = {
                "observed_pixels": 10,
                "counts": {"vegetation": 8, "other": 2},
                "dominant": "vegetation",
            }
            materials_path.write_text(json.dumps(materials), encoding="utf-8")
            newer = image_path.stat().st_mtime + 1.0
            for path in (texture_path, obstacle_path, materials_path):
                os.utime(path, (newer, newer))

            with patch.multiple(
                self.bridge,
                MAP_DIRECTORY=str(root),
                MAP_TEXTURE_PATH=str(texture_path),
                MAP_OBSTACLE_TEXTURE_PATH=str(obstacle_path),
                MAP_MATERIALS_PATH=str(materials_path),
            ):
                payload = node.load_map_payload()

            self.assertEqual(
                base64.b64decode(payload["obstacle_texture_base64"]),
                b"obstacle-png",
            )
            self.assertEqual(payload["obstacle_materials"], materials)


if __name__ == "__main__":
    unittest.main()
