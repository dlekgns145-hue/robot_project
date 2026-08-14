#!/usr/bin/env python3
"""Authenticated GUI gateway that follows a DHCP robot by MAC or hostname."""

from __future__ import annotations

import json
import math
import os
import signal
import socket
import threading
import time
from pathlib import Path
from typing import Any

from config import env_float, env_int
from map_inbox import MapInbox
from map_payload import load_configured_map_payload
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
ROBOT_RESPONSE_TIMEOUT = env_float("ROBOT_RESPONSE_TIMEOUT_SEC", 8.0)
ROBOT_RESPONSE_MAX_BYTES = env_int("ROBOT_RESPONSE_MAX_BYTES", 16 * 1024 * 1024)
OPERATIONS_DB_PATH = os.getenv(
    "OPERATIONS_DB_PATH", "/var/lib/robot-control-v2/operations.sqlite3"
)
ROBOT_NAME = os.getenv("ROBOT_NAME", "Yahboom Robot 1")
MAP_DIRECTORY = os.getenv("MAP_DIRECTORY", "/opt/robot-control/maps")
MAP_NAME = os.getenv("MAP_NAME", "orchard_map")

STOP_EVENT = threading.Event()


class CommandRejectedError(ValueError):
    """Raised when the robot bridge returns a logical rejection (ok=False) without a socket fault."""


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))



def legacy_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Translate the v2 GUI protocol to the existing Yahboom bridge protocol."""
    command_type = payload.get("type", "command")
    if command_type == "emergency_stop":
        return {"linear": 0.0, "angular": 0.0, "emergency_stop": 1}
    if command_type == "navigation_cancel":
        return {"type": "navigation_cancel"}
    if command_type == "map_request":
        return {"type": "map_request"}
    if command_type in {
        "mapping_start",
        "mapping_stop",
        "mapping_save",
        "mapping_preview",
        "follow_start",
        "follow_stop",
    }:
        return {"type": command_type}
    if command_type == "navigate":
        x = float(payload["x"])
        y = float(payload["y"])
        yaw = float(payload.get("yaw", 0.0))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("navigation coordinates must be finite")
        if abs(x) > 100.0 or abs(y) > 100.0 or abs(yaw) > math.pi:
            raise ValueError("navigation coordinates are outside allowed bounds")
        return {"type": "navigate", "x": x, "y": y, "yaw": yaw}
    if command_type not in {"command", "ping"}:
        raise ValueError(f"unsupported command type: {command_type}")

    if command_type == "ping":
        return {"heartbeat": 1}

    linear = float(payload.get("linear", 0.0))
    angular = float(payload.get("angular", 0.0))
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
        self,
        locator: RobotLocator,
        operations: OperationsStore | None = None,
        map_inbox: MapInbox | None = None,
    ) -> None:
        self.locator = locator
        self.operations = operations
        self.map_inbox = map_inbox
        self.lock = threading.Lock()
        self.connection: socket.socket | None = None
        self.resolution: Resolution | None = None
        self.last_error = "not connected"
        self.last_sent_at = 0.0
        self.applied_linear = 0.0
        self.applied_angular = 0.0
        self._response_buffer = bytearray()
        self._robot_response: dict[str, Any] = {}
        self._last_map_transfer_key = ""
        self._last_map_transfer_attempt_key = ""
        self._next_map_transfer_attempt_at = 0.0
        self._map_transfer_status: dict[str, Any] = {
            "state": "idle",
            "message": "waiting for a completed robot map",
        }
        self._map_root = Path(MAP_DIRECTORY)
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
        self._response_buffer.clear()
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
        mapping_snapshot: dict[str, Any] | None = None
        with self.lock:
            if self.connection is None:
                return
            if time.monotonic() - self.last_sent_at < ROBOT_HEARTBEAT_INTERVAL:
                return
            try:
                # Keep the TCP link alive without renewing the GUI motor-command
                # lease. New robot bridges ignore this marker; older bridges
                # safely interpret the missing velocities as zero.
                self._exchange_unlocked({"heartbeat": 1})
                mapping = self._robot_response.get("mapping")
                if isinstance(mapping, dict):
                    mapping_snapshot = dict(mapping)
                if self.operations is not None:
                    resolution = self.resolution
                    self.operations.set_robot_state(
                        True, ip="" if resolution is None else resolution.ip
                    )
            except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as caught:
                error = caught
                ip = "" if self.resolution is None else self.resolution.ip
                self.last_error = str(caught)
                self._close_unlocked(invalidate=False)
                self.locator.invalidate(clear_runtime=False)
                self._reconnect_event.set()
                self.applied_linear = 0.0
                self.applied_angular = 0.0
        if error is None:
            if mapping_snapshot is not None:
                self._stage_completed_map(mapping_snapshot)
            self._deliver_pending_corrected_map()
            return
        if error is not None and self.operations is not None:
            self.operations.set_robot_state(
                False, ip=ip, detail=f"heartbeat lost: {error}"
            )

    def _stage_completed_map(self, mapping: dict[str, Any]) -> None:
        """Fetch one stable map pair after mapping has stopped and saved."""

        if self.map_inbox is None:
            return
        if bool(mapping.get("enabled", False)):
            return
        saved_map = str(mapping.get("saved_map") or "")
        save_sequence = int(mapping.get("save_sequence", 0))
        state = str(mapping.get("state") or "")
        saved_signal = bool(saved_map and save_sequence > 0)
        persisted_map_probe = state in {"idle", "stopped", "completed"}
        if not saved_signal and not persisted_map_probe:
            return
        transfer_key = (
            json.dumps(
                {
                    "saved_map": saved_map,
                    "save_sequence": save_sequence,
                    "map_sequence": int(mapping.get("map_sequence", 0)),
                    "known_area_m2": (mapping.get("map_quality") or {}).get(
                        "known_area_m2"
                    ),
                },
                sort_keys=True,
            )
            if saved_signal
            else "persisted-map-probe"
        )
        if transfer_key == self._last_map_transfer_key:
            return
        now = time.monotonic()
        if (
            transfer_key == self._last_map_transfer_attempt_key
            and now < self._next_map_transfer_attempt_at
        ):
            return
        self._last_map_transfer_attempt_key = transfer_key
        self._next_map_transfer_attempt_at = now + 15.0
        try:
            with self.lock:
                if self.connection is None:
                    return
                response = self._exchange_unlocked(
                    {"type": "map_request", "map_variant": "raw"}
                )
            map_payload = response.get("map")
            if not isinstance(map_payload, dict):
                raise ValueError("robot did not return its completed map")
            job_id = self.map_inbox.stage(map_payload, mapping)
            self._last_map_transfer_key = transfer_key
            self._next_map_transfer_attempt_at = 0.0
            self._map_transfer_status = {
                "state": "duplicate" if job_id is None else "queued",
                "message": (
                    "completed map already exists on the server"
                    if job_id is None
                    else "completed map queued for Ubuntu post-processing"
                ),
                "job_id": job_id,
            }
            print(self._map_transfer_status["message"], flush=True)
        except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as error:
            # A post-processing transfer failure must not turn into a motor or
            # robot-connectivity failure. The next heartbeat retries it.
            self._map_transfer_status = {
                "state": "retrying",
                "message": str(error),
            }
            print(f"completed map transfer deferred: {error}", flush=True)

    def _deliver_pending_corrected_map(self) -> None:
        outbox = self._map_root / "postprocess-outbox"
        pending = sorted(outbox.glob("*.json"))
        if not pending:
            return
        bundle_path = pending[0]
        try:
            bundle = json.loads(bundle_path.read_text(encoding="utf-8"))
            if not isinstance(bundle, dict):
                raise ValueError("corrected map outbox item is not an object")
            with self.lock:
                if self.connection is None:
                    return
                response = self._exchange_unlocked(
                    {"type": "map_install", "bundle": bundle}
                )
            installed = response.get("map_install")
            if not isinstance(installed, dict) or not bool(
                installed.get("installed", False)
            ):
                raise ValueError("robot did not acknowledge corrected map installation")
            delivered = self._map_root / "postprocess-delivered"
            delivered.mkdir(parents=True, exist_ok=True)
            os.replace(bundle_path, delivered / bundle_path.name)
            self._map_transfer_status = {
                "state": "installed_on_robot",
                "message": (
                    "corrected map and transformed pose installed; "
                    "start the navigation runtime to use this exact bundle"
                ),
                "job_id": str(installed.get("job_id") or bundle.get("job_id") or ""),
            }
            print(self._map_transfer_status["message"], flush=True)
        except CommandRejectedError as error:
            self._map_transfer_status = {
                "state": "return_retrying",
                "message": str(error),
                "job_id": bundle_path.stem,
            }
        except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as error:
            self._map_transfer_status = {
                "state": "return_retrying",
                "message": str(error),
                "job_id": bundle_path.stem,
            }

    def _exchange_unlocked(self, command: dict[str, Any]) -> dict[str, Any]:
        if self.connection is None:
            raise ConnectionError("robot bridge is not connected")
        encoded = json.dumps(command, separators=(",", ":")).encode() + b"\n"
        self.connection.sendall(encoded)
        deadline = time.monotonic() + max(2.0, ROBOT_RESPONSE_TIMEOUT)
        while b"\n" not in self._response_buffer:
            if time.monotonic() > deadline:
                raise TimeoutError("robot bridge response timeout")
            try:
                chunk = self.connection.recv(65_536)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionResetError("robot bridge closed the connection")
            self._response_buffer.extend(chunk)
            if len(self._response_buffer) > ROBOT_RESPONSE_MAX_BYTES:
                raise ValueError(
                    f"robot response exceeded {ROBOT_RESPONSE_MAX_BYTES} bytes"
                )
        raw, _, remainder = self._response_buffer.partition(b"\n")
        self._response_buffer = bytearray(remainder)
        response = json.loads(raw)
        if not isinstance(response, dict):
            raise ValueError("robot bridge response must be an object")
        self._robot_response = response
        self.last_sent_at = time.monotonic()
        if not response.get("ok", False):
            raise CommandRejectedError(
                str(response.get("error") or "robot command rejected")
            )
        return response

    def send(self, command: dict[str, Any]) -> bool:
        mapping_snapshot: dict[str, Any] | None = None
        with self.lock:
            if self.connection is None:
                self._reconnect_event.set()
                self.applied_linear = 0.0
                self.applied_angular = 0.0
                return False
            try:
                self._exchange_unlocked(command)
                mapping = self._robot_response.get("mapping")
                if isinstance(mapping, dict):
                    mapping_snapshot = dict(mapping)
                self.applied_linear = float(command.get("linear", 0.0))
                self.applied_angular = float(command.get("angular", 0.0))
                self.last_error = ""
            except CommandRejectedError as error:
                self.last_error = str(error)
                self.applied_linear = 0.0
                self.applied_angular = 0.0
                return False
            except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as error:
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

        # Active GUIs send commands more frequently than the idle heartbeat.
        # Process map handoff after every successful robot exchange so GUI
        # traffic cannot indefinitely suppress completed-map upload or the
        # corrected bundle's return trip. Keep this outside the relay lock:
        # both helpers perform their own robot exchange.
        if mapping_snapshot is not None:
            self._stage_completed_map(mapping_snapshot)
        self._deliver_pending_corrected_map()
        return True

    def safe_stop(self) -> None:
        with self.lock:
            if self.connection is not None:
                try:
                    self._exchange_unlocked(
                        {"linear": 0.0, "angular": 0.0, "emergency_stop": 1}
                    )
                except (OSError, TimeoutError, ValueError, json.JSONDecodeError):
                    pass
            self.applied_linear = 0.0
            self.applied_angular = 0.0

    def release_manual_control(self) -> None:
        """Release a GUI manual-control lease without canceling autonomous work."""
        with self.lock:
            if self.connection is not None:
                try:
                    # The robot bridge ignores ordinary manual commands while
                    # Nav2 owns the motors.  An emergency stop here would also
                    # cancel autonomous mapping whenever a GUI goes idle.
                    self._exchange_unlocked({"linear": 0.0, "angular": 0.0})
                except (OSError, TimeoutError, ValueError, json.JSONDecodeError):
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
            connected = bool(sent and self.connection is not None)
            last_robot_ip = None if resolution is None else resolution.ip
            last_method = None if resolution is None else resolution.method
            status = {
                "type": "status",
                "gateway_connected": True,
                "robot_connected": connected,
                # Do not present a stale neighbor-table result as the robot's
                # current DHCP address after its TCP bridge is unreachable.
                "robot_ip": last_robot_ip if connected else None,
                "last_robot_ip": last_robot_ip,
                "robot_mac": self.locator.robot_mac or None,
                "discovery_method": last_method if connected else None,
                "last_discovery_method": last_method,
                "robot_error": self.last_error or None,
                "lidar_ok": None,
                "front_distance": None,
                "avoid_state": "ROBOT_LOCAL" if sent else "ROBOT_DISCONNECTED",
                "applied_linear": round(self.applied_linear, 3),
                "applied_angular": round(self.applied_angular, 3),
            }
            navigation = self._robot_response.get("navigation")
            if isinstance(navigation, dict):
                status["navigation"] = dict(navigation)
            map_payload = self._robot_response.get("map")
            if isinstance(map_payload, dict):
                status["map"] = dict(map_payload)
            mapping = self._robot_response.get("mapping")
            if isinstance(mapping, dict):
                status["mapping"] = dict(mapping)
            command_result = self._robot_response.get("command_result")
            if isinstance(command_result, dict):
                status["command_result"] = dict(command_result)
            status["map_postprocess"] = dict(self._map_transfer_status)
            return status


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
                    if payload.get("type") == "map_request":
                        # Server-compute maps live beside the gateway. Keep a
                        # robot fallback so older all-in-one deployments still
                        # work without configuration changes.
                        try:
                            sent = relay.send({"heartbeat": 1})
                            response = relay.status(sent)
                            response["map"] = load_configured_map_payload()
                        except (OSError, ValueError, json.JSONDecodeError):
                            sent = relay.send(command)
                            response = relay.status(sent)
                    else:
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
        relay.release_manual_control()


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
    relay = RobotRelay(
        locator,
        operations,
        MapInbox(MAP_DIRECTORY, MAP_NAME),
    )
    relay.start()

    def handle_client(connection: socket.socket, address: Any) -> None:
        with connection:
            try:
                serve_gui(connection, relay, operations)
            except (ConnectionError, OSError, ValueError) as error:
                print(f"GUI disconnected: {error}", flush=True)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((LISTEN_HOST, COMMAND_PORT))
        server.listen(16)
        server.settimeout(0.5)
        print(f"robot gateway listening through TCP :{COMMAND_PORT}", flush=True)
        while not STOP_EVENT.is_set():
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            print(f"GUI connected: {address}", flush=True)
            # RobotRelay already guards its shared state with a lock (it was
            # designed for concurrent access), but this loop used to call
            # serve_gui() inline and block on it -- accept() never ran again
            # until that one client fully disconnected. A GUI reconnect
            # (after any network blip) or a second client then had to wait
            # for the old, possibly still-lingering connection to be reaped
            # by serve_gui's own idle timeout before it could even be
            # accepted, which looked like the gateway silently ignoring
            # commands. Serve each connection on its own thread instead.
            threading.Thread(
                target=handle_client,
                args=(connection, address),
                daemon=True,
            ).start()

    relay.shutdown()
    operations.close()


if __name__ == "__main__":
    main()
