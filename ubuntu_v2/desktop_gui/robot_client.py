"""Fixed-rate command/heartbeat client for the robot bridge."""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from typing import Any

from PySide6.QtCore import QThread, Signal


class RobotClient(QThread):
    connection_changed = Signal(bool, str)
    status_received = Signal(dict)
    operations_received = Signal(dict)
    map_received = Signal(dict)
    log_message = Signal(str)
    STOP_BURST_COUNT = 5

    def __init__(
        self,
        host: str,
        port: int,
        token: str = "",
        robot_host: str = "raspberrypi.local",
        parent: object | None = None,
        admin_enabled: bool = False,
    ) -> None:
        super().__init__(parent)
        self.host = host
        self.port = port
        self.token = token
        self.robot_host = robot_host.strip()
        self.admin_enabled = admin_enabled
        self._admin_revision = -1
        self._robot_ip_hint = ""
        self._robot_resolved_at = 0.0
        self._stop_event = threading.Event()
        self._shutdown_requested = threading.Event()
        self._command_lock = threading.Lock()
        self._pending_commands: deque[dict[str, Any]] = deque()
        self._command: dict[str, Any] = {
            "type": "command",
            "linear": 0.0,
            "angular": 0.0,
        }

    def set_command(
        self, linear: float, angular: float, servo_pan: int | None = None
    ) -> None:
        if self._shutdown_requested.is_set():
            return
        command: dict[str, Any] = {
            "type": "command",
            "linear": float(linear),
            "angular": float(angular),
        }
        if servo_pan is not None:
            command["servo_pan"] = int(servo_pan)
        with self._command_lock:
            self._command = command

    def emergency_stop(self) -> None:
        with self._command_lock:
            self._pending_commands.clear()
            self._command = {"type": "emergency_stop"}

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> None:
        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._pending_commands.append(
                {"type": "navigate", "x": float(x), "y": float(y), "yaw": float(yaw)}
            )
            self._command = {"type": "ping"}

    def cancel_navigation(self) -> None:
        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._pending_commands.append({"type": "navigation_cancel"})
            self._command = {"type": "ping"}

    def request_map(self) -> None:
        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._pending_commands.append({"type": "map_request"})

    def mapping_action(self, action: str) -> None:
        command_type = f"mapping_{action}"
        if command_type not in {
            "mapping_start",
            "mapping_stop",
            "mapping_save",
            "mapping_preview",
        }:
            raise ValueError(f"unsupported mapping action: {action}")
        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._pending_commands.append({"type": command_type})
            self._command = {"type": "ping"}

    def follow_action(self, action: str) -> None:
        command_type = f"follow_{action}"
        if command_type not in {"follow_start", "follow_stop"}:
            raise ValueError(f"unsupported follow action: {action}")
        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._pending_commands.append({"type": command_type})
            self._command = {"type": "ping"}

    def release_control(self) -> None:
        """Keep status polling alive without publishing a motor command."""

        if self._shutdown_requested.is_set():
            return
        with self._command_lock:
            self._command = {"type": "ping"}

    def stop(self) -> None:
        # The worker sends and acknowledges a stop burst before it exits.
        self.emergency_stop()
        self._shutdown_requested.set()

    def _emergency_payload(self) -> bytes:
        command: dict[str, Any] = {"type": "emergency_stop"}
        if self.token:
            command["token"] = self.token
        return json.dumps(command, separators=(",", ":")).encode() + b"\n"

    def _send_stop_burst(
        self, connection: socket.socket, buffer: bytearray
    ) -> bytearray:
        payload = self._emergency_payload()
        for _ in range(self.STOP_BURST_COUNT):
            connection.sendall(payload)
            _, buffer = self._read_line(connection, buffer)
        return buffer

    def _best_effort_stop(self) -> None:
        try:
            with socket.create_connection(
                (self.host, self.port), timeout=0.5
            ) as connection:
                connection.settimeout(0.2)
                connection.sendall(self._emergency_payload() * self.STOP_BURST_COUNT)
        except OSError:
            # The robot-side command timeout remains the final safety fallback.
            pass

    def _current_command(self) -> dict[str, Any]:
        with self._command_lock:
            command = dict(
                self._pending_commands.popleft()
                if self._pending_commands
                else self._command
            )
        if self.token:
            command["token"] = self.token
        if self.robot_host and time.monotonic() - self._robot_resolved_at >= 5.0:
            try:
                addresses = socket.getaddrinfo(
                    self.robot_host,
                    None,
                    family=socket.AF_INET,
                    type=socket.SOCK_STREAM,
                )
                self._robot_ip_hint = addresses[0][4][0] if addresses else ""
            except socket.gaierror:
                self._robot_ip_hint = ""
            self._robot_resolved_at = time.monotonic()
        if self._robot_ip_hint:
            command["robot_ip_hint"] = self._robot_ip_hint
        if self.admin_enabled:
            command["admin_revision"] = self._admin_revision
        return command

    def _read_line(
        self, connection: socket.socket, buffer: bytearray
    ) -> tuple[bytes, bytearray]:
        # A saved orchard map can include a compressed camera texture.  Allow
        # enough time for that one large response while preserving the regular
        # 10 Hz heartbeat cadence after it arrives.
        deadline = time.monotonic() + 10.0
        while b"\n" not in buffer:
            if self._stop_event.is_set():
                raise ConnectionAbortedError("client stopping")
            if time.monotonic() > deadline:
                raise TimeoutError("robot status timeout")
            try:
                chunk = connection.recv(65_536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionResetError("robot closed the connection")
            buffer.extend(chunk)
            if len(buffer) > 32 * 1024 * 1024:
                raise ValueError("gateway response exceeded 32 MiB")
        line, _, remainder = buffer.partition(b"\n")
        return bytes(line), bytearray(remainder)

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._shutdown_requested.is_set():
                self._best_effort_stop()
                self._stop_event.set()
                break
            try:
                self.connection_changed.emit(False, f"{self.host}:{self.port} 연결 중")
                with socket.create_connection(
                    (self.host, self.port), timeout=2.0
                ) as connection:
                    connection.settimeout(0.2)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    self.connection_changed.emit(True, "로봇 제어 서버 연결됨")
                    buffer = bytearray()
                    next_send = time.monotonic()
                    while not self._stop_event.is_set():
                        if self._shutdown_requested.is_set():
                            buffer = self._send_stop_burst(connection, buffer)
                            self._stop_event.set()
                            break
                        payload = (
                            json.dumps(
                                self._current_command(), separators=(",", ":")
                            ).encode()
                            + b"\n"
                        )
                        connection.sendall(payload)
                        line, buffer = self._read_line(connection, buffer)
                        response = json.loads(line)
                        is_direct_status = bool(response.get("ok")) and any(
                            key in response
                            for key in ("navigation", "mapping", "map")
                        )
                        if response.get("type") == "status" or is_direct_status:
                            operations = response.pop("operations", None)
                            map_payload = response.pop("map", None)
                            if isinstance(map_payload, dict):
                                self.map_received.emit(map_payload)
                            if isinstance(operations, dict):
                                revision = operations.get("revision")
                                if revision is not None:
                                    self._admin_revision = int(revision)
                                if "robots" in operations:
                                    self.operations_received.emit(operations)
                            self.status_received.emit(response)
                        elif (
                            response.get("type") == "error"
                            or response.get("ok") is False
                        ):
                            raise ValueError(
                                response.get("message")
                                or response.get("error")
                                or "robot rejected command"
                            )
                        else:
                            self.log_message.emit(str(response))

                        next_send += 0.1
                        delay = next_send - time.monotonic()
                        if delay > 0:
                            self._stop_event.wait(delay)
                        else:
                            next_send = time.monotonic()
            except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as error:
                if self._shutdown_requested.is_set():
                    self._stop_event.set()
                elif not self._stop_event.is_set():
                    self.connection_changed.emit(False, f"연결 끊김: {error}")
                    self._stop_event.wait(1.0)
        self.connection_changed.emit(False, "연결 종료")
