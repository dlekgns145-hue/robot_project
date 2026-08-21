#!/usr/bin/env python3
"""Publish the existing Arduino weight-classifier Serial output to ROS 2.

The Arduino remains the sole owner of the HX711, the 2.8 g classification rule,
and the sorting servo.  This bridge is intentionally read-only: it never sends
commands to the Arduino and does not change the uploaded sketch.
"""

from __future__ import annotations

import errno
import json
import os
import re
import termios
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger


WEIGHT_PATTERN = re.compile(r"측정된\s*무게\s*:\s*([-+]?\d+(?:\.\d+)?)\s*g", re.I)


def classify_serial_line(line: str) -> tuple[str | None, float | None, str | None]:
    """Return ``(phase, grams, classification)`` for one sketch output line."""
    normalized = " ".join((line or "").strip().split())
    match = WEIGHT_PATTERN.search(normalized)
    if match:
        return "measured", float(match.group(1)), None
    if "오른쪽" in normalized and "이상" in normalized:
        return "sorting", None, "above"
    if "왼쪽" in normalized and "미만" in normalized:
        return "sorting", None, "below"
    if "물건 감지" in normalized:
        return "settling", None, None
    if "치워지길" in normalized:
        return "waiting_removal", None, None
    if "대기 상태로 복귀" in normalized or "물건을 올려주세요" in normalized:
        return "idle", 0.0, "normal"
    if "무게 분류 시스템 시작" in normalized:
        return "starting", None, None
    return None, None, None


class ArduinoLoadcellBridge(Node):
    def __init__(self) -> None:
        super().__init__("loadcell_guard")
        self.declare_parameter("serial_device", "/dev/loadcell-arduino")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("low_threshold_g", 0.8)
        self.declare_parameter("high_threshold_g", 2.8)
        self.declare_parameter("reconnect_sec", 1.0)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.weight_publisher = self.create_publisher(
            Float32, "/loadcell/weight_grams", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/loadcell_guard/status", status_qos
        )
        self.overweight_publisher = self.create_publisher(
            Bool, "/loadcell_guard/above_threshold", 10
        )
        self.underweight_publisher = self.create_publisher(
            Bool, "/loadcell_guard/below_threshold", 10
        )
        self.create_service(Trigger, "/loadcell_guard/tare", self._tare_service)

        self._serial_fd: Optional[int] = None
        self._buffer = bytearray()
        self._last_open_attempt = 0.0
        self._last_line_at = 0.0
        self._last_grams: Optional[float] = 0.0
        self._classification = "normal"
        self._phase = "disconnected"
        self._last_line = ""
        self.create_timer(0.05, self._poll_serial)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info("Arduino USB loadcell bridge 시작됨")

    def _open_serial(self) -> None:
        now = time.monotonic()
        reconnect_sec = max(
            0.2, float(self.get_parameter("reconnect_sec").value)
        )
        if now - self._last_open_attempt < reconnect_sec:
            return
        self._last_open_attempt = now
        device = str(self.get_parameter("serial_device").value)
        baud = int(self.get_parameter("baud").value)
        if baud != 9600:
            self.get_logger().warn(
                f"Arduino sketch는 9600 baud임; 요청값 {baud} 대신 9600 사용"
            )
        try:
            fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attributes = termios.tcgetattr(fd)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attributes[3] = 0
            attributes[4] = termios.B9600
            attributes[5] = termios.B9600
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIFLUSH)
        except OSError as error:
            self.get_logger().warn(
                f"Arduino Serial 연결 대기: {device}: {error}",
                throttle_duration_sec=5.0,
            )
            return
        self._serial_fd = fd
        self._buffer.clear()
        self._phase = "connected"
        self.get_logger().info(f"Arduino Serial 연결됨: {device} @ 9600")
        self._publish_status()

    def _close_serial(self, reason: str) -> None:
        if self._serial_fd is not None:
            try:
                os.close(self._serial_fd)
            except OSError:
                pass
        self._serial_fd = None
        self._buffer.clear()
        self._phase = "disconnected"
        self.get_logger().warn(reason, throttle_duration_sec=5.0)
        self._publish_status()

    def _poll_serial(self) -> None:
        if self._serial_fd is None:
            self._open_serial()
            return
        try:
            chunk = os.read(self._serial_fd, 4096)
        except BlockingIOError:
            return
        except OSError as error:
            if error.errno in {errno.EAGAIN, errno.EWOULDBLOCK}:
                return
            self._close_serial(f"Arduino Serial 연결 끊김: {error}")
            return
        if not chunk:
            return
        self._buffer.extend(chunk)
        if len(self._buffer) > 16_384:
            del self._buffer[:-4096]
        while b"\n" in self._buffer:
            raw_line, _, remainder = self._buffer.partition(b"\n")
            self._buffer = bytearray(remainder)
            line = raw_line.rstrip(b"\r").decode("utf-8", errors="replace").strip()
            if line:
                self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        phase, grams, classification = classify_serial_line(line)
        self._last_line_at = time.monotonic()
        self._last_line = line[:160]
        if phase is not None:
            self._phase = phase
        if grams is not None:
            self._last_grams = grams
            weight = Float32()
            weight.data = float(grams)
            self.weight_publisher.publish(weight)
            low = float(self.get_parameter("low_threshold_g").value)
            high = float(self.get_parameter("high_threshold_g").value)
            if grams >= high:
                self._classification = "above"
            elif grams >= low:
                self._classification = "below"
            else:
                self._classification = "normal"
        if classification is not None:
            self._classification = classification
        self.get_logger().info(f"Arduino: {line}")
        self._publish_status()

    def _publish_status(self) -> None:
        connected = self._serial_fd is not None
        age = (
            time.monotonic() - self._last_line_at
            if self._last_line_at > 0.0
            else None
        )
        state = self._classification if connected else "unavailable"
        above = Bool()
        above.data = state == "above"
        self.overweight_publisher.publish(above)
        below = Bool()
        below.data = state == "below"
        self.underweight_publisher.publish(below)
        status = String()
        status.data = json.dumps(
            {
                "connected": connected,
                "source": "arduino_usb",
                "grams": self._last_grams if connected else None,
                "state": state,
                "phase": self._phase,
                "low_threshold_g": float(
                    self.get_parameter("low_threshold_g").value
                ),
                "high_threshold_g": float(
                    self.get_parameter("high_threshold_g").value
                ),
                "age_s": None if age is None else round(age, 3),
                "last_line": self._last_line,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)

    def _tare_service(self, request, response):
        response.success = False
        response.message = "영점 조절은 Arduino 재시작 시 기존 스케치가 수행합니다."
        return response

    def destroy_node(self):
        if self._serial_fd is not None:
            try:
                os.close(self._serial_fd)
            except OSError:
                pass
            self._serial_fd = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ArduinoLoadcellBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
