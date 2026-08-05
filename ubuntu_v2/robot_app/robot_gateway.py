#!/usr/bin/env python3
"""Authenticated GUI gateway that follows a DHCP robot by MAC or hostname."""

from __future__ import annotations

import json
import os
import signal
import socket
import threading
import time
from typing import Any

from config import env_float, env_int
from operations_store import OperationsStore
from robot_locator import Resolution, RobotLocator


LISTEN_HOST = "0.0.0.0"
COMMAND_PORT = env_int("COMMAND_PORT", 9999)
COMMAND_TOKEN = os.getenv("COMMAND_TOKEN", "")
CLIENT_IDLE_TIMEOUT = env_float("CLIENT_IDLE_TIMEOUT_SEC", 3.0)
MAX_LINEAR_SPEED = env_float("MAX_LINEAR_SPEED", 0.5)
MAX_ANGULAR_SPEED = env_float("MAX_ANGULAR_SPEED", 0.8)
ROBOT_PORT = env_int("ROBOT_PORT", 9999)
ROBOT_CONNECT_TIMEOUT = env_float("ROBOT_CONNECT_TIMEOUT_SEC", 1.5)
ROBOT_RECONNECT_INTERVAL = env_float("ROBOT_RECONNECT_INTERVAL_SEC", 10.0)
ROBOT_HEARTBEAT_INTERVAL = env_float("ROBOT_HEARTBEAT_INTERVAL_SEC", 1.0)
OPERATIONS_DB_PATH = os.getenv(
    "OPERATIONS_DB_PATH", "/var/lib/robot-control-v2/operations.sqlite3"
)
ROBOT_NAME = os.getenv("ROBOT_NAME", "Yahboom Robot 1")

STOP_EVENT = threading.Event()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def legacy_payload(payload: dict[str, Any]) -> dict[str, float | int]:
    """Translate the v2 GUI protocol to the existing Yahboom bridge protocol."""
    command_type = payload.get("type", "command")
    if command_type == "emergency_stop":
        return {"linear": 0.0, "angular": 0.0, "emergency_stop": 1}
    if command_type not in {"command", "ping"}:
        raise ValueError(f"unsupported command type: {command_type}")

    linear = 0.0 if command_type == "ping" else float(payload.get("linear", 0.0))
    angular = 0.0 if command_type == "ping" else float(payload.get("angular", 0.0))
    command: dict[str, float | int] = {
        "linear": clamp(linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED),
        "angular": clamp(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED),
    }
    servo_pan = payload.get("servo_pan")
    if servo_pan is not None:
        command["servo_pan"] = int(clamp(float(servo_pan), -60.0, 60.0))
    return command


class RobotRelay:
    def __init__(
        self, locator: RobotLocator, operations: OperationsStore | None = None
    ) -> None:
        self.locator = locator
        self.operations = operations
        self.lock = threading.Lock()
        self.connection: socket.socket | None = None
        self.resolution: Resolution | None = None
        self.last_error = "not connected"
        self.last_sent_at = 0.0
        self.applied_linear = 0.0
        self.applied_angular = 0.0
        self._shutdown_event = threading.Event()
        self._reconnect_event = threading.Event()
        self._worker = threading.Thread(target=self._connection_loop, daemon=True)

    def start(self) -> None:
        self._worker.start()

    def set_ip_hint(self, value: str) -> None:
        disconnected = False
        with self.lock:
            previous_ip = None if self.resolution is None else self.resolution.ip
        self.locator.set_runtime_ip(value, "gui-hostname")
        if previous_ip and previous_ip != value:
            with self.lock:
                self._close_unlocked(invalidate=False)
                disconnected = True
        if disconnected and self.operations is not None:
            self.operations.set_robot_state(
                False, ip=previous_ip or "", detail="robot IP changed"
            )
        self._reconnect_event.set()

    def _close_unlocked(self, *, invalidate: bool) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except OSError:
                pass
        self.connection = None
        if invalidate:
            self.resolution = None
            self.locator.invalidate()

    def _connection_loop(self) -> None:
        while not self._shutdown_event.is_set():
            with self.lock:
                connected = self.connection is not None
            if connected:
                if time.monotonic() - self.last_sent_at >= ROBOT_HEARTBEAT_INTERVAL:
                    self._send_idle_heartbeat()
                self._reconnect_event.wait(0.25)
                self._reconnect_event.clear()
                continue

            try:
                resolution = self.locator.resolve(force=True)
                connection = socket.create_connection(
                    (resolution.ip, ROBOT_PORT), timeout=ROBOT_CONNECT_TIMEOUT
                )
                connection.settimeout(ROBOT_CONNECT_TIMEOUT)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                if hasattr(socket, "TCP_KEEPIDLE"):
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
                if hasattr(socket, "TCP_KEEPINTVL"):
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
                if hasattr(socket, "TCP_KEEPCNT"):
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                with self.lock:
                    if self._shutdown_event.is_set():
                        connection.close()
                        return
                    self.connection = connection
                    self.resolution = resolution
                    self.last_error = ""
                    self.last_sent_at = time.monotonic()
                print(
                    f"robot bridge connected: {resolution.ip}:{ROBOT_PORT} "
                    f"via {resolution.method}",
                    flush=True,
                )
                if self.operations is not None:
                    self.operations.set_robot_state(
                        True,
                        ip=resolution.ip,
                        detail=f"connected via {resolution.method}",
                    )
            except (OSError, RuntimeError, ValueError) as error:
                with self.lock:
                    self.last_error = str(error)
                if self.operations is not None:
                    self.operations.set_robot_state(
                        False, detail=f"connection failed: {error}"
                    )
                self.locator.invalidate(clear_runtime=True)
                self._reconnect_event.wait(ROBOT_RECONNECT_INTERVAL)
                self._reconnect_event.clear()

    def _send_idle_heartbeat(self) -> None:
        """Keep availability tracking alive without moving the robot."""
        error: OSError | None = None
        ip = ""
        with self.lock:
            if self.connection is None:
                return
            if time.monotonic() - self.last_sent_at < ROBOT_HEARTBEAT_INTERVAL:
                return
            try:
                self.connection.sendall(b'{"linear":0.0,"angular":0.0}\n')
                self.last_sent_at = time.monotonic()
                if self.operations is not None:
                    resolution = self.resolution
                    self.operations.set_robot_state(
                        True, ip="" if resolution is None else resolution.ip
                    )
                return
            except OSError as caught:
                error = caught
                ip = "" if self.resolution is None else self.resolution.ip
                self.last_error = str(caught)
                self._close_unlocked(invalidate=False)
                self.locator.invalidate(clear_runtime=False)
                self._reconnect_event.set()
                self.applied_linear = 0.0
                self.applied_angular = 0.0
        if error is not None and self.operations is not None:
            self.operations.set_robot_state(
                False, ip=ip, detail=f"heartbeat lost: {error}"
            )

    def send(self, command: dict[str, float | int]) -> bool:
        encoded = json.dumps(command, separators=(",", ":")).encode() + b"\n"
        with self.lock:
            if self.connection is None:
                self._reconnect_event.set()
                self.applied_linear = 0.0
                self.applied_angular = 0.0
                return False
            try:
                self.connection.sendall(encoded)
                self.last_sent_at = time.monotonic()
                self.applied_linear = float(command.get("linear", 0.0))
                self.applied_angular = float(command.get("angular", 0.0))
                self.last_error = ""
                return True
            except OSError as error:
                self.last_error = str(error)
                # A bridge restart does not imply that the robot's DHCP IP changed.
                # Preserve the authenticated GUI hint for an immediate reconnect.
                self._close_unlocked(invalidate=False)
                self.locator.invalidate(clear_runtime=False)
                self._reconnect_event.set()
                self.applied_linear = 0.0
                self.applied_angular = 0.0
                if self.operations is not None:
                    self.operations.set_robot_state(
                        False,
                        ip="" if self.resolution is None else self.resolution.ip,
                        detail=f"command connection lost: {error}",
                    )
                return False

    def safe_stop(self) -> None:
        stop_message = b'{"linear":0.0,"angular":0.0,"emergency_stop":1}\n'
        with self.lock:
            if self.connection is not None:
                try:
                    for _ in range(3):
                        self.connection.sendall(stop_message)
                except OSError:
                    pass
            self.applied_linear = 0.0
            self.applied_angular = 0.0

    def shutdown(self) -> None:
        self._shutdown_event.set()
        self._reconnect_event.set()
        self.safe_stop()
        with self.lock:
            resolution = self.resolution
            self._close_unlocked(invalidate=False)
        if self.operations is not None:
            self.operations.set_robot_state(
                False,
                ip="" if resolution is None else resolution.ip,
                detail="gateway shutdown",
            )
        self._worker.join(timeout=1.0)

    def status(self, sent: bool) -> dict[str, Any]:
        with self.lock:
            resolution = self.resolution
            return {
                "type": "status",
                "gateway_connected": True,
                "robot_connected": bool(sent and self.connection is not None),
                "robot_ip": None if resolution is None else resolution.ip,
                "robot_mac": self.locator.robot_mac or None,
                "discovery_method": None if resolution is None else resolution.method,
                "robot_error": self.last_error or None,
                "lidar_ok": None,
                "front_distance": None,
                "avoid_state": "ROBOT_LOCAL" if sent else "ROBOT_DISCONNECTED",
                "applied_linear": round(self.applied_linear, 3),
                "applied_angular": round(self.applied_angular, 3),
            }


def serve_gui(
    connection: socket.socket,
    relay: RobotRelay,
    operations: OperationsStore | None = None,
) -> None:
    connection.settimeout(0.5)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    buffer = bytearray()
    last_data_at = time.monotonic()
    try:
        while not STOP_EVENT.is_set():
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                if time.monotonic() - last_data_at > CLIENT_IDLE_TIMEOUT:
                    return
                continue
            if not chunk:
                return
            last_data_at = time.monotonic()
            buffer.extend(chunk)
            if len(buffer) > 65_536:
                raise ValueError("command buffer exceeded 64 KiB")

            while b"\n" in buffer:
                raw_line, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                if not raw_line.strip():
                    continue
                try:
                    payload = json.loads(raw_line)
                    if not isinstance(payload, dict):
                        raise ValueError("JSON command must be an object")
                    if COMMAND_TOKEN and payload.get("token") != COMMAND_TOKEN:
                        raise ValueError("invalid command token")
                    robot_ip_hint = payload.get("robot_ip_hint")
                    if robot_ip_hint:
                        relay.set_ip_hint(str(robot_ip_hint))
                    command = legacy_payload(payload)
                    sent = relay.send(command)
                    response = relay.status(sent)
                    if operations is not None and "admin_revision" in payload:
                        try:
                            client_revision = int(payload["admin_revision"])
                        except (TypeError, ValueError):
                            client_revision = -1
                        if client_revision != operations.revision():
                            response["operations"] = operations.snapshot()
                except (json.JSONDecodeError, TypeError, ValueError) as error:
                    response = {"type": "error", "message": str(error)}
                connection.sendall(
                    json.dumps(response, separators=(",", ":")).encode() + b"\n"
                )
    finally:
        relay.safe_stop()


def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: STOP_EVENT.set())
    locator = RobotLocator(
        robot_mac=os.getenv("ROBOT_MAC", ""),
        robot_host=os.getenv("ROBOT_HOST", "raspberrypi.local"),
        robot_ip=os.getenv("ROBOT_IP", ""),
        interface=os.getenv("ROBOT_INTERFACE", ""),
        discovery_cidr=os.getenv("ROBOT_DISCOVERY_CIDR", ""),
        cache_seconds=env_float("ROBOT_DISCOVERY_CACHE_SEC", 30.0),
        command_timeout=env_float("ROBOT_DISCOVERY_TIMEOUT_SEC", 8.0),
    )
    robot_id = os.getenv("ROBOT_ID", "").strip()
    if not robot_id:
        robot_id = locator.robot_mac or locator.robot_host or "robot-1"
    operations = OperationsStore(
        OPERATIONS_DB_PATH,
        robot_id=robot_id,
        robot_name=ROBOT_NAME,
    )
    operations.mark_gateway_started()
    relay = RobotRelay(locator, operations)
    relay.start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((LISTEN_HOST, COMMAND_PORT))
        server.listen(4)
        server.settimeout(0.5)
        print(f"robot gateway listening through TCP :{COMMAND_PORT}", flush=True)
        while not STOP_EVENT.is_set():
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            print(f"GUI connected: {address}", flush=True)
            with connection:
                try:
                    serve_gui(connection, relay, operations)
                except (ConnectionError, OSError, ValueError) as error:
                    print(f"GUI disconnected: {error}", flush=True)

    relay.shutdown()
    operations.close()


if __name__ == "__main__":
    main()
