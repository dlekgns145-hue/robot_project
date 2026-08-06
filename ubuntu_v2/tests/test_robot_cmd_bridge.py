from __future__ import annotations

import importlib.util
import sys
import threading
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
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = type("SetBool", (), {})
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.action": rclpy_action,
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
        self.assertEqual(node.pub.messages, [])

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


if __name__ == "__main__":
    unittest.main()
