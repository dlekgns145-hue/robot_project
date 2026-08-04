"""Fixed-rate command/heartbeat client for the robot bridge."""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any

from PySide6.QtCore import QThread, Signal


class RobotClient(QThread):
    connection_changed = Signal(bool, str)
    status_received = Signal(dict)
    log_message = Signal(str)
    STOP_BURST_COUNT = 5

    def __init__(
        self,
        host: str,
        port: int,
        token: str = "",
        robot_host: str = "raspberrypi.local",
        parent: object | None = None,
    ) -> None:
        super().__init__(parent)
        self.host = host
        self.port = port
        self.token = token
        self.robot_host = robot_host.strip()
        self._robot_ip_hint = ""
        self._robot_resolved_at = 0.0
        self._stop_event = threading.Event()
        self._shutdown_requested = threading.Event()
        self._command_lock = threading.Lock()
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
            self._command = {"type": "emergency_stop"}

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
            command = dict(self._command)
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
        return command

    def _read_line(
        self, connection: socket.socket, buffer: bytearray
    ) -> tuple[bytes, bytearray]:
        deadline = time.monotonic() + 0.8
        while b"\n" not in buffer:
            if self._stop_event.is_set():
                raise ConnectionAbortedError("client stopping")
            if time.monotonic() > deadline:
                raise TimeoutError("robot status timeout")
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionResetError("robot closed the connection")
            buffer.extend(chunk)
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
                    self.connection_changed.emit(True, "Ubuntu VM 게이트웨이 연결됨")
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
                        if response.get("type") == "status":
                            self.status_received.emit(response)
                        elif response.get("type") == "error":
                            raise ValueError(
                                response.get("message", "robot rejected command")
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
