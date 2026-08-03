#!/usr/bin/env python3
"""ROS 2 command bridge with watchdog, LiDAR safety, and JSON telemetry."""

from __future__ import annotations

import json
import math
import os
import signal
import socket
import threading
import time
from collections import deque
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

from config import env_bool, env_float, env_int


HOST = "0.0.0.0"
PORT = env_int("COMMAND_PORT", 9999)
COMMAND_TIMEOUT = env_float("COMMAND_TIMEOUT_SEC", 0.7)
COMMAND_TOKEN = os.getenv("COMMAND_TOKEN", "")
CLIENT_IDLE_TIMEOUT = env_float("CLIENT_IDLE_TIMEOUT_SEC", 3.0)
LIDAR_TIMEOUT = env_float("LIDAR_TIMEOUT_SEC", 1.0)
LIDAR_REQUIRED = env_bool("LIDAR_REQUIRED", True)
MAX_LINEAR_SPEED = env_float("MAX_LINEAR_SPEED", 0.5)
MAX_ANGULAR_SPEED = env_float("MAX_ANGULAR_SPEED", 0.8)

OBSTACLE_STOP_DISTANCE = env_float("OBSTACLE_STOP_DISTANCE_M", 0.40)
SAFE_CLEAR_DISTANCE = env_float("SAFE_CLEAR_DISTANCE_M", 0.65)
BACKUP_TRIGGER_DISTANCE = env_float("BACKUP_TRIGGER_DISTANCE_M", 0.30)
BACKUP_TARGET_DISTANCE = env_float("BACKUP_TARGET_DISTANCE_M", 0.55)
BACKUP_MAX_TIME = env_float("BACKUP_MAX_TIME_SEC", 3.0)
BACKUP_SPEED = env_float("BACKUP_SPEED", -0.20)
AVOID_LINEAR_SPEED = env_float("AVOID_LINEAR_SPEED", 0.12)
AVOID_ANGULAR_SPEED = env_float("AVOID_ANGULAR_SPEED", 0.35)
FRONT_ANGLE_RANGE_DEG = env_float("FRONT_ANGLE_RANGE_DEG", 30.0)
SIDE_ANGLE_MIN_DEG = env_float("SIDE_ANGLE_MIN_DEG", 30.0)
SIDE_ANGLE_MAX_DEG = env_float("SIDE_ANGLE_MAX_DEG", 70.0)
LIDAR_MIN_VALID_RANGE = env_float("LIDAR_MIN_VALID_RANGE", 0.02)

STUCK_TIMEOUT = env_float("STUCK_TIMEOUT_SEC", 4.0)
ESCAPE_ANGULAR_SPEED = env_float("ESCAPE_ANGULAR_SPEED", 0.4)
ESCAPE_TURN_TIME = env_float("ESCAPE_TURN_TIME_SEC", 2.5)
SERVO_TILT_DEFAULT = env_int("SERVO_TILT_DEFAULT", -60)
SERVO_PAN_MIN = env_int("SERVO_PAN_MIN", -60)
SERVO_PAN_MAX = env_int("SERVO_PAN_MAX", 60)

STOP_EVENT = threading.Event()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class RobotServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_control_v2")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pan_pub = self.create_publisher(Int32, "/servo_s1", 10)
        self.tilt_pub = self.create_publisher(Int32, "/servo_s2", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self.set_initial_tilt)

        self.lock = threading.Lock()
        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.desired_servo_pan: int | None = None
        self.last_command_at = 0.0
        self.last_scan_at = 0.0
        self.front_min_distance = math.inf
        self.front_blocked = False
        self.left_history: deque[float] = deque(maxlen=5)
        self.right_history: deque[float] = deque(maxlen=5)
        self.avoid_state = "NORMAL"
        self.state_started_at = time.monotonic()
        self.avoid_cycle_started_at = self.state_started_at
        self.emergency_latched = False
        self.last_pan_sent: int | None = None
        self.tilt_initialized = False
        self.last_periodic_log_at = 0.0
        self.applied_linear = 0.0
        self.applied_angular = 0.0
        self.scan_geometry: (
            tuple[
                tuple[float, float, int],
                tuple[int, ...],
                tuple[int, ...],
                tuple[int, ...],
            ]
            | None
        ) = None
        self.get_logger().info(f"robot control v2 listening through TCP :{PORT}")

    def set_initial_tilt(self) -> None:
        if self.tilt_initialized:
            return
        message = Int32()
        message.data = SERVO_TILT_DEFAULT
        self.tilt_pub.publish(message)
        self.tilt_initialized = True

    def scan_callback(self, message: LaserScan) -> None:
        front_indices, left_indices, right_indices = self._scan_indices(message)
        front_min = math.inf
        left_sum = right_sum = 0.0
        left_count = right_count = 0

        for index in front_indices:
            distance = message.ranges[index]
            if math.isfinite(distance) and distance >= LIDAR_MIN_VALID_RANGE:
                front_min = min(front_min, distance)
        for index in left_indices:
            distance = message.ranges[index]
            if math.isfinite(distance) and distance >= LIDAR_MIN_VALID_RANGE:
                left_sum += distance
                left_count += 1
        for index in right_indices:
            distance = message.ranges[index]
            if math.isfinite(distance) and distance >= LIDAR_MIN_VALID_RANGE:
                right_sum += distance
                right_count += 1

        with self.lock:
            self.front_min_distance = front_min
            self.front_blocked = front_min < OBSTACLE_STOP_DISTANCE
            self.last_scan_at = time.monotonic()
            if left_count:
                self.left_history.append(left_sum / left_count)
            if right_count:
                self.right_history.append(right_sum / right_count)

    def _scan_indices(
        self, message: LaserScan
    ) -> tuple[tuple[int, ...], tuple[int, ...], tuple[int, ...]]:
        key = (message.angle_min, message.angle_increment, len(message.ranges))
        if self.scan_geometry is not None and self.scan_geometry[0] == key:
            return self.scan_geometry[1:]

        front_limit = math.radians(FRONT_ANGLE_RANGE_DEG)
        side_min = math.radians(SIDE_ANGLE_MIN_DEG)
        side_max = math.radians(SIDE_ANGLE_MAX_DEG)
        front: list[int] = []
        left: list[int] = []
        right: list[int] = []
        for index in range(len(message.ranges)):
            angle = message.angle_min + index * message.angle_increment
            if -front_limit <= angle <= front_limit:
                front.append(index)
            elif side_min <= angle <= side_max:
                right.append(index)
            elif -side_max <= angle <= -side_min:
                left.append(index)
        self.scan_geometry = (key, tuple(front), tuple(left), tuple(right))
        return self.scan_geometry[1:]

    def accept_command(self, payload: dict[str, Any]) -> None:
        if COMMAND_TOKEN and payload.get("token") != COMMAND_TOKEN:
            raise ValueError("invalid command token")
        command_type = payload.get("type", "command")
        now = time.monotonic()
        with self.lock:
            if command_type == "emergency_stop":
                self.desired_linear = 0.0
                self.desired_angular = 0.0
                self.emergency_latched = True
                self.last_command_at = now
                return
            if command_type == "ping":
                self.last_command_at = now
                return

            linear = float(payload.get("linear", 0.0))
            angular = float(payload.get("angular", 0.0))
            self.desired_linear = clamp(linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            self.desired_angular = clamp(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            servo_pan = payload.get("servo_pan")
            if servo_pan is not None:
                self.desired_servo_pan = int(
                    clamp(float(servo_pan), SERVO_PAN_MIN, SERVO_PAN_MAX)
                )
            self.emergency_latched = False
            self.last_command_at = now

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            front = self.front_min_distance
            return {
                "type": "status",
                "connected": now - self.last_command_at <= COMMAND_TIMEOUT,
                "command_age": round(now - self.last_command_at, 3),
                "scan_age": None
                if self.last_scan_at == 0
                else round(now - self.last_scan_at, 3),
                "lidar_ok": self.last_scan_at > 0
                and now - self.last_scan_at <= LIDAR_TIMEOUT,
                "front_distance": None if not math.isfinite(front) else round(front, 3),
                "avoid_state": self.avoid_state,
                "emergency_stop": self.emergency_latched,
                "applied_linear": round(self.applied_linear, 3),
                "applied_angular": round(self.applied_angular, 3),
            }

    def _change_state(self, state: str, now: float) -> None:
        if self.avoid_state == state:
            return
        self.avoid_state = state
        self.state_started_at = now
        self.get_logger().warn(f"avoid state: {state}")

    @staticmethod
    def _average(values: deque[float]) -> float | None:
        return sum(values) / len(values) if values else None

    def _desired_twist(self, twist: Twist) -> None:
        twist.linear.x = self.desired_linear
        twist.angular.z = self.desired_angular

    def control_loop(self) -> None:
        now = time.monotonic()
        twist = Twist()

        with self.lock:
            command_fresh = now - self.last_command_at <= COMMAND_TIMEOUT
            lidar_fresh = (
                self.last_scan_at > 0 and now - self.last_scan_at <= LIDAR_TIMEOUT
            )

            if not command_fresh or self.emergency_latched:
                self._change_state("NORMAL", now)
            elif LIDAR_REQUIRED and not lidar_fresh and self.desired_linear > 0:
                self._change_state("LIDAR_STOP", now)
                twist.angular.z = self.desired_angular
            else:
                if self.avoid_state == "LIDAR_STOP":
                    self._change_state("NORMAL", now)

                if (
                    self.avoid_state == "NORMAL"
                    and self.front_blocked
                    and self.desired_linear > 0
                ):
                    self.avoid_cycle_started_at = now
                    state = (
                        "BACKING_UP"
                        if self.front_min_distance < BACKUP_TRIGGER_DISTANCE
                        else "AVOIDING"
                    )
                    self._change_state(state, now)

                if self.avoid_state in {"BACKING_UP", "AVOIDING"}:
                    if now - self.avoid_cycle_started_at > STUCK_TIMEOUT:
                        self._change_state("ESCAPE_TURN", now)

                elapsed = now - self.state_started_at
                if self.avoid_state == "NORMAL":
                    self._desired_twist(twist)
                elif self.avoid_state == "BACKING_UP":
                    if (
                        self.front_min_distance < BACKUP_TARGET_DISTANCE
                        and elapsed < BACKUP_MAX_TIME
                    ):
                        twist.linear.x = BACKUP_SPEED
                    else:
                        self._change_state("AVOIDING", now)
                elif self.avoid_state == "AVOIDING":
                    if self.front_min_distance >= SAFE_CLEAR_DISTANCE:
                        self._change_state("NORMAL", now)
                        self._desired_twist(twist)
                    elif self.front_min_distance < BACKUP_TRIGGER_DISTANCE:
                        self._change_state("BACKING_UP", now)
                    else:
                        left = self._average(self.left_history)
                        right = self._average(self.right_history)
                        if left is not None or right is not None:
                            turn_left = right is None or (
                                left is not None and left > right
                            )
                            twist.angular.z = (
                                AVOID_ANGULAR_SPEED
                                if turn_left
                                else -AVOID_ANGULAR_SPEED
                            )
                            if self.front_min_distance >= OBSTACLE_STOP_DISTANCE + 0.1:
                                twist.linear.x = AVOID_LINEAR_SPEED
                elif self.avoid_state == "ESCAPE_TURN":
                    if elapsed < ESCAPE_TURN_TIME:
                        twist.angular.z = ESCAPE_ANGULAR_SPEED
                    else:
                        self._change_state("NORMAL", now)

            servo_pan = self.desired_servo_pan
            self.applied_linear = twist.linear.x
            self.applied_angular = twist.angular.z

        self.cmd_pub.publish(twist)
        if servo_pan is not None and servo_pan != self.last_pan_sent:
            message = Int32()
            message.data = servo_pan
            self.pan_pub.publish(message)
            self.last_pan_sent = servo_pan

    def emergency_stop(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)


def serve_client(connection: socket.socket, node: RobotServiceNode) -> None:
    connection.settimeout(0.5)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    buffer = bytearray()
    last_data_at = time.monotonic()
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
                node.accept_command(payload)
                response = json.dumps(node.status(), separators=(",", ":")) + "\n"
            except (json.JSONDecodeError, TypeError, ValueError) as error:
                response = json.dumps({"type": "error", "message": str(error)}) + "\n"
            connection.sendall(response.encode())


def socket_server(node: RobotServiceNode) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(4)
        server.settimeout(0.5)
        while not STOP_EVENT.is_set():
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            with connection:
                try:
                    serve_client(connection, node)
                except (ConnectionError, OSError, ValueError) as error:
                    print(f"client {address} closed: {error}", flush=True)


def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: STOP_EVENT.set())
    rclpy.init()
    node = RobotServiceNode()
    server_thread = threading.Thread(target=socket_server, args=(node,), daemon=True)
    server_thread.start()
    try:
        while rclpy.ok() and not STOP_EVENT.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        STOP_EVENT.set()
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()
        server_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
