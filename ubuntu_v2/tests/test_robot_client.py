from __future__ import annotations

import json
import socket
import sys
import threading
import unittest
from pathlib import Path


GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
sys.path.insert(0, str(GUI_DIR))

from robot_client import RobotClient  # noqa: E402


class RobotClientShutdownTests(unittest.TestCase):
    def test_release_control_switches_status_loop_to_ping(self) -> None:
        client = RobotClient("127.0.0.1", 9999, robot_host="")
        client.set_command(0.2, 0.1)

        client.release_control()

        self.assertEqual(client._current_command(), {"type": "ping"})

    def test_navigation_and_map_requests_are_sent_once_before_ping(self) -> None:
        client = RobotClient("127.0.0.1", 9999, robot_host="")

        client.request_map()
        client.navigate_to(1.2, -0.4, 0.5)

        self.assertEqual(client._current_command(), {"type": "map_request"})
        self.assertEqual(
            client._current_command(),
            {"type": "navigate", "x": 1.2, "y": -0.4, "yaw": 0.5},
        )
        self.assertEqual(client._current_command(), {"type": "ping"})

    def test_shutdown_sends_stop_burst_after_motion(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        commands: list[dict[str, object]] = []
        motion_received = threading.Event()

        def gateway() -> None:
            connection, _ = listener.accept()
            connection.settimeout(2.0)
            buffer = bytearray()
            with connection:
                while True:
                    chunk = connection.recv(4096)
                    if not chunk:
                        return
                    buffer.extend(chunk)
                    while b"\n" in buffer:
                        raw, _, remainder = buffer.partition(b"\n")
                        buffer = bytearray(remainder)
                        command = json.loads(raw)
                        commands.append(command)
                        if float(command.get("linear", 0.0)) > 0.0:
                            motion_received.set()
                        connection.sendall(b'{"type":"status"}\n')
                        stop_count = sum(
                            item.get("type") == "emergency_stop" for item in commands
                        )
                        if stop_count >= RobotClient.STOP_BURST_COUNT:
                            return

        server_thread = threading.Thread(target=gateway)
        server_thread.start()
        client = RobotClient("127.0.0.1", port, robot_host="")
        client.set_command(0.2, 0.0)
        client.start()
        self.assertTrue(motion_received.wait(2.0))

        client.stop()
        client.set_command(0.4, 0.2)  # A late vision signal must not undo shutdown.
        self.assertTrue(client.wait(3000))
        server_thread.join(timeout=2.0)
        listener.close()

        emergency_indices = [
            index
            for index, command in enumerate(commands)
            if command.get("type") == "emergency_stop"
        ]
        self.assertGreaterEqual(len(emergency_indices), RobotClient.STOP_BURST_COUNT)
        self.assertFalse(
            any(
                float(command.get("linear", 0.0)) != 0.0
                for command in commands[emergency_indices[0] :]
            )
        )


if __name__ == "__main__":
    unittest.main()
