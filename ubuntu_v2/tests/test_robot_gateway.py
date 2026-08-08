from __future__ import annotations

import sys
import json
import socket
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

APP_DIR = Path(__file__).resolve().parents[1] / "robot_app"
sys.path.insert(0, str(APP_DIR))

import robot_gateway  # noqa: E402
from robot_gateway import RobotRelay, legacy_payload  # noqa: E402
from robot_locator import (  # noqa: E402
    RobotLocator,
    normalize_mac,
    parse_arp_scan,
    parse_neighbor_table,
)


class RobotLocatorTests(unittest.TestCase):
    def test_mac_is_normalized(self) -> None:
        self.assertEqual(normalize_mac("DC-A6-32-01-02-03"), "dc:a6:32:01:02:03")

    def test_invalid_mac_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            normalize_mac("not-a-mac")

    def test_neighbor_table_returns_matching_reachable_ip(self) -> None:
        output = "\n".join(
            [
                "172.30.1.18 dev enp0s1 FAILED",
                "172.30.1.42 dev enp0s1 lladdr dc:a6:32:01:02:03 REACHABLE",
            ]
        )
        self.assertEqual(
            parse_neighbor_table(output, "dc:a6:32:01:02:03"), "172.30.1.42"
        )

    def test_arp_scan_returns_matching_ip(self) -> None:
        output = "172.30.1.77\tdc:a6:32:01:02:03\tRaspberry Pi Trading Ltd\n"
        self.assertEqual(parse_arp_scan(output, "DC-A6-32-01-02-03"), "172.30.1.77")

    def test_authenticated_runtime_hint_can_follow_mdns_result(self) -> None:
        locator = RobotLocator(robot_mac="dc:a6:32:01:02:03")
        locator.set_runtime_ip("172.30.1.99", "gui-hostname")
        resolution = locator.resolve(force=True)
        self.assertEqual(
            (resolution.ip, resolution.method), ("172.30.1.99", "gui-hostname")
        )

    def test_runtime_hint_can_survive_transient_bridge_disconnect(self) -> None:
        locator = RobotLocator(robot_mac="dc:a6:32:01:02:03")
        locator.set_runtime_ip("172.30.1.18", "gui-hostname")
        locator.invalidate(clear_runtime=False)
        resolution = locator.resolve(force=True)
        self.assertEqual(
            (resolution.ip, resolution.method), ("172.30.1.18", "gui-hostname")
        )


class GatewayProtocolTests(unittest.TestCase):
    def test_command_is_clamped_for_legacy_bridge(self) -> None:
        command = legacy_payload(
            {"type": "command", "linear": 99, "angular": -99, "servo_pan": 70}
        )
        self.assertEqual(command, {"linear": 0.5, "angular": -0.8, "servo_pan": 60})

    def test_emergency_stop_becomes_zero_velocity(self) -> None:
        self.assertEqual(
            legacy_payload({"type": "emergency_stop"}),
            {"linear": 0.0, "angular": 0.0, "emergency_stop": 1},
        )

    def test_ping_releases_motor_control(self) -> None:
        self.assertEqual(legacy_payload({"type": "ping"}), {"heartbeat": 1})

    def test_navigation_commands_are_validated_and_forwarded(self) -> None:
        self.assertEqual(
            legacy_payload({"type": "navigate", "x": 1, "y": -2, "yaw": 0.5}),
            {"type": "navigate", "x": 1.0, "y": -2.0, "yaw": 0.5},
        )
        self.assertEqual(
            legacy_payload({"type": "navigation_cancel"}),
            {"type": "navigation_cancel"},
        )
        self.assertEqual(legacy_payload({"type": "map_request"}), {"type": "map_request"})

    def test_mapping_commands_are_forwarded(self) -> None:
        for command_type in (
            "mapping_start",
            "mapping_stop",
            "mapping_save",
            "mapping_preview",
        ):
            self.assertEqual(
                legacy_payload({"type": command_type}), {"type": command_type}
            )

    def test_relay_connects_and_sends_to_legacy_robot(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        received: list[dict[str, float]] = []

        def robot_server() -> None:
            connection, _ = listener.accept()
            with connection:
                raw = connection.recv(1024).splitlines()[0]
                received.append(json.loads(raw))
                connection.sendall(
                    b'{"ok":true,"navigation":{"state":"idle","active":false}}\n'
                )

        server_thread = threading.Thread(target=robot_server)
        server_thread.start()
        locator = RobotLocator(robot_ip="127.0.0.1")
        relay = RobotRelay(locator)
        with patch.object(robot_gateway, "ROBOT_PORT", port):
            relay.start()
            deadline = time.monotonic() + 2.0
            while relay.connection is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(relay.send({"linear": 0.2, "angular": -0.1}))
            server_thread.join(timeout=2.0)
            relay.shutdown()
        listener.close()
        self.assertEqual(received, [{"linear": 0.2, "angular": -0.1}])

    def test_idle_heartbeat_does_not_claim_motor_control(self) -> None:
        class RecordingConnection:
            def __init__(self) -> None:
                self.payloads: list[bytes] = []

            def sendall(self, payload: bytes) -> None:
                self.payloads.append(payload)

            def recv(self, _size: int) -> bytes:
                return b'{"ok":true,"navigation":{"state":"idle","active":false}}\n'

        relay = RobotRelay(RobotLocator(robot_ip="127.0.0.1"))
        connection = RecordingConnection()
        relay.connection = connection  # type: ignore[assignment]
        relay.last_sent_at = 0.0

        relay._send_idle_heartbeat()

        self.assertEqual(connection.payloads, [b'{"heartbeat":1}\n'])


if __name__ == "__main__":
    unittest.main()
