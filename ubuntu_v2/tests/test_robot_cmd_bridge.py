from __future__ import annotations

import base64
import importlib.util
import json
import math
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


class NullLogger:
    def error(self, *args, **kwargs) -> None:
        pass

    def warn(self, *args, **kwargs) -> None:
        pass

    def info(self, *args, **kwargs) -> None:
        pass


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
    std_msgs_msg.Float32 = type("Float32", (), {})
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
    sys.path.insert(0, str(path.parent))
    try:
        with patch.dict(sys.modules, stubs):
            spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
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
        node.follow_enabled = False
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
        node.front_blocked = False
        node.avoid_state_start = time.time()
        node.avoid_cycle_start = time.time()
        node.left_history = []
        node.right_history = []
        node._last_camera_scan_at = 0.0
        node.camera_front_min_dist = float("inf")
        node._remote_nav_state = "idle"
        node._remote_nav_sending_since = 0.0
        node.get_logger = lambda: NullLogger()
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

    def test_follow_command_drives_through_the_gui_motor_path(self) -> None:
        node = self.make_node()
        node.desired_linear = 0.0
        node.desired_angular = 0.0
        node._last_servo_pan_sent = None
        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.1

        node.follow_cmd_callback(command)
        node.control_loop()

        self.assertAlmostEqual(node.desired_linear, 0.2)
        self.assertAlmostEqual(node.desired_angular, 0.1)
        self.assertEqual(len(node.pub.messages), 1)
        self.assertAlmostEqual(node.pub.messages[0].linear.x, 0.2)

    def test_navigation_mode_rejects_follow_commands_too(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.desired_linear = 0.0
        node.desired_angular = 0.0
        node._last_servo_pan_sent = None
        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.1

        node.follow_cmd_callback(command)

        self.assertEqual(node.desired_linear, 0.0)
        self.assertEqual(node.desired_angular, 0.0)

    def test_follow_command_with_non_finite_velocity_is_discarded(self) -> None:
        node = self.make_node()
        node.desired_linear = 0.0
        node.desired_angular = 0.0
        node._last_servo_pan_sent = None
        command = Twist()
        command.linear.x = float("nan")
        command.angular.z = 0.1

        node.follow_cmd_callback(command)

        self.assertEqual(node.desired_linear, 0.0)
        self.assertEqual(node.desired_angular, 0.0)

    def test_pure_rotation_is_boosted_over_motor_deadzone(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(0.0, 0.08)

        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.18)

    def test_small_driving_steering_noise_is_removed(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(0.08, -0.02)

        self.assertAlmostEqual(linear, 0.08)
        self.assertEqual(angular, 0.0)

    def test_small_translation_is_boosted_over_motor_deadzone(self) -> None:
        linear, angular = self.bridge.shape_server_velocity(0.013, -0.009)

        self.assertAlmostEqual(linear, 0.04)
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

    def test_fresh_server_command_renews_active_navigation_lease(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.navigation_lease_active = True
        node.last_navigation_lease_at = 1.0
        command = Twist()
        command.linear.x = 0.013

        node.server_cmd_callback(command)

        self.assertGreater(node.last_navigation_lease_at, 1.0)
        self.assertAlmostEqual(node.server_linear, 0.04)

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

    def test_side_camera_obstacle_is_left_to_nav2_costmap(self) -> None:
        node = self.make_node()
        scan = types.SimpleNamespace(
            angle_min=math.radians(-35.0),
            angle_increment=math.radians(1.0),
            range_min=0.22,
            ranges=[math.inf] * 71,
        )
        # A desk leg near the right edge of the former +/-30 degree bridge
        # sector must remain in Nav2's camera costmap without globally
        # disabling forward motion at the motor bridge.
        scan.ranges[7] = 0.24  # -28 degrees

        node.camera_scan_callback(scan)

        self.assertTrue(math.isinf(node.camera_front_min_dist))

    def test_central_camera_obstacle_remains_in_hard_stop_corridor(self) -> None:
        node = self.make_node()
        scan = types.SimpleNamespace(
            angle_min=math.radians(-35.0),
            angle_increment=math.radians(1.0),
            range_min=0.22,
            ranges=[math.inf] * 71,
        )
        scan.ranges[35] = 0.24

        node.camera_scan_callback(scan)

        self.assertAlmostEqual(node.camera_front_min_dist, 0.24)

    def test_critical_camera_obstacle_allows_pure_recovery_rotation(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_angular = 0.18
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)
        self.assertAlmostEqual(node.pub.messages[0].angular.z, 0.18)

    def test_critical_camera_obstacle_creeps_forward_instead_of_stopping(self) -> None:
        # A full stop here treated every close camera reading -- real or a
        # false positive -- as an absolute wall, and combined with Nav2's
        # own spin collision check also refusing rotation in the same tight
        # quarters, left no automatic way out even where a careful manual
        # driver had already proven the space passable (2026-08-19).
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22

        node.control_loop()

        self.assertAlmostEqual(
            node.pub.messages[0].linear.x, self.bridge.CAMERA_CRITICAL_CREEP_SPEED
        )
        self.assertEqual(node._navigation_safety_reason, "camera_critical_creep")

    def test_critical_camera_obstacle_creeps_backward_when_reversing(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = -0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22

        node.control_loop()

        self.assertAlmostEqual(
            node.pub.messages[0].linear.x, -self.bridge.CAMERA_CRITICAL_CREEP_SPEED
        )

    def test_critical_camera_creep_still_stops_for_a_close_lidar_front_reading(
        self,
    ) -> None:
        # The camera's own near-field reading is unreliable enough to creep
        # through rather than trust outright, but LiDAR is not -- it must
        # keep its full hard stop even while the camera gate is creeping.
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22
        node.front_min_dist = 0.30

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)
        self.assertEqual(node._navigation_safety_reason, "lidar_front")

    def test_critical_camera_creep_still_stops_for_a_close_lidar_rear_reading(
        self,
    ) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = -0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22
        node.rear_min_dist = 0.30

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)
        self.assertEqual(node._navigation_safety_reason, "lidar_rear")

    def test_safety_status_reason_stays_clear_for_a_side_obstacle(self) -> None:
        # autonomous_mapping.py's safety-block timer only starts for the
        # blocking reasons; a side obstacle must stay "clear" here or the
        # mapper would think a forward request it never actually blocked
        # is being held back.
        node = self.make_node()
        scan = types.SimpleNamespace(
            angle_min=math.radians(-35.0),
            angle_increment=math.radians(1.0),
            range_min=0.22,
            ranges=[math.inf] * 71,
        )
        scan.ranges[7] = 0.24  # -28 degrees, outside the +/-12 hard-stop corridor
        node.camera_scan_callback(scan)
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = time.monotonic()

        node.control_loop()

        self.assertEqual(node._navigation_safety_reason, "clear")
        self.assertAlmostEqual(node.pub.messages[0].linear.x, 0.1)

    def test_safety_status_reason_flags_a_central_obstacle(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = 0.1
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.4

        node.control_loop()

        self.assertEqual(node._navigation_safety_reason, "camera_front")
        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_safety_status_reason_stays_clear_during_pure_rotation(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_angular = 0.18
        node.last_server_cmd_at = time.monotonic()
        node._last_camera_scan_at = time.monotonic()
        node.camera_front_min_dist = 0.22

        node.control_loop()

        self.assertEqual(node._navigation_safety_reason, "clear")

    def test_rear_lidar_obstacle_blocks_server_reverse_motion(self) -> None:
        node = self.make_node()
        node.navigation_mode = True
        node.server_linear = -0.08
        node.last_server_cmd_at = time.monotonic()
        node.rear_min_dist = 0.3

        node.control_loop()

        self.assertEqual(node.pub.messages[0].linear.x, 0.0)

    def test_measured_rear_housing_reflection_is_ignored(self) -> None:
        self.assertTrue(
            self.bridge.is_rear_housing_reflection(
                self.bridge.math.radians(-168.0), 0.12
            )
        )
        self.assertFalse(
            self.bridge.is_rear_housing_reflection(
                self.bridge.math.radians(-168.0), 0.50
            )
        )

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

    def test_navigate_home_uses_map_bound_saved_pose(self) -> None:
        homes = []
        node = types.SimpleNamespace(
            navigate_home=lambda: homes.append(True)
            or {"x": 1.0, "y": -2.0, "yaw": 0.5, "map_sha256": "a" * 64},
            navigation_snapshot=lambda: {"state": "sending", "active": True},
        )

        response = self.bridge.handle_socket_command(node, {"type": "navigate_home"})

        self.assertEqual(homes, [True])
        self.assertEqual(response["home"]["x"], 1.0)
        self.assertTrue(response["navigation"]["active"])

    def test_soft_pause_never_starts_a_goal(self) -> None:
        pauses = []
        node = types.SimpleNamespace(
            soft_pause=lambda: pauses.append(True) or "robot paused",
            navigation_snapshot=lambda: {"state": "canceled", "active": False},
        )

        response = self.bridge.handle_socket_command(node, {"type": "soft_pause"})

        self.assertEqual(pauses, [True])
        self.assertEqual(response["command_result"]["type"], "soft_pause")
        self.assertFalse(response["navigation"]["active"])

    def test_home_pose_must_match_active_map(self) -> None:
        pose = {"x": 1.0, "y": 2.0, "yaw": 7.0, "map_sha256": "a" * 64}

        validated = self.bridge.validated_home_pose(pose, "a" * 64)

        self.assertAlmostEqual(validated["x"], 1.0)
        self.assertGreaterEqual(validated["yaw"], -self.bridge.math.pi)
        self.assertLessEqual(validated["yaw"], self.bridge.math.pi)
        with self.assertRaisesRegex(ValueError, "different map"):
            self.bridge.validated_home_pose(pose, "b" * 64)

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

    def test_apple_detection_status_is_returned(self) -> None:
        node = types.SimpleNamespace(
            navigation_snapshot=lambda: {"state": "idle", "active": False},
            apple_detection_snapshot=lambda: {
                "connected": True,
                "model_ready": True,
                "state": "healthy",
                "healthy_count": 1,
                "damaged_count": 0,
            },
        )

        response = self.bridge.handle_socket_command(
            node, {"heartbeat": True, "linear": 0.0, "angular": 0.0}
        )

        self.assertEqual(response["apple_detection"]["state"], "healthy")
        self.assertEqual(response["apple_detection"]["healthy_count"], 1)

    def test_map_payload_ignores_visual_layer_files(self) -> None:
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
            ):
                payload = node.load_map_payload()

            self.assertEqual(payload["occupancy_source"], "lidar_slam_only")
            self.assertTrue(payload["navigation_safe"])
            self.assertNotIn("texture_base64", payload)
            self.assertNotIn("obstacle_texture_base64", payload)
            self.assertNotIn("obstacle_materials", payload)


if __name__ == "__main__":
    unittest.main()
