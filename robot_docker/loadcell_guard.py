#!/usr/bin/env python3
"""Read an HX711-amplified load cell over Pi5 GPIO and detect weight thresholds.

The load cell is wired directly to the Raspberry Pi 5's GPIO header (DT/SCK to
HX711), not through the ESP32/micro-ROS-agent path used by the motors, IMU and
encoders. The vendor ESP32 firmware is closed and not part of this repository,
so it cannot be extended to read a new sensor -- unlike /scan or /imu, this
sensor's only integration point is the Pi5 itself.

Like camera_obstacle_guard.py, this node needs no compute server: threshold
detection must keep working even when the robot is standalone (mode A in
HANDOFF.md), so it runs locally and only reports its state upstream.
"""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

HX711_SETTLE_SEC = 0.0000001  # HX711 needs the clock line held >0.2us per edge


def raw_to_grams(raw: int, tare_offset: int, grams_per_count: float) -> float:
    """Convert an HX711 24-bit signed reading to grams.

    ``grams_per_count`` must come from a physical calibration (raw reading
    delta divided by a known reference weight) -- there is no vendor-published
    value, the same situation as the robot's wheel diameter (see
    HANDOFF.md 2절): only a real measurement on the actual load cell is
    trustworthy.
    """

    if grams_per_count == 0.0:
        raise ValueError("grams_per_count must not be zero (not calibrated)")
    return (raw - tare_offset) * grams_per_count


def classify_weight(
    grams: Optional[float], low_threshold_g: float, high_threshold_g: float
) -> str:
    """Classify a weight reading against the configured band.

    Returns one of ``"unavailable"``, ``"below"``, ``"normal"``, ``"above"``.
    ``low_threshold_g``/``high_threshold_g`` are independent so callers can
    detect "적재량 부족"(below) and "과적"(above) as separate conditions, per
    the "일정무게 이상,이하 검출" requirement.
    """

    if grams is None:
        return "unavailable"
    if grams < low_threshold_g:
        return "below"
    if grams > high_threshold_g:
        return "above"
    return "normal"


class HX711:
    """Minimal bit-banged HX711 reader.

    Uses gpiozero so the Pi5's RP1 GPIO chip is supported through its lgpio
    pin factory (the classic RPi.GPIO package does not work on Pi5). Timing
    is not interrupt-safe -- acceptable here because weight only needs to be
    sampled at a few Hz, not read at the LiDAR/IMU control-loop rate.
    """

    def __init__(self, dt_pin: int, sck_pin: int, gain_pulses: int = 1):
        from gpiozero import DigitalInputDevice, DigitalOutputDevice

        self._dt = DigitalInputDevice(dt_pin, pull_up=False)
        self._sck = DigitalOutputDevice(sck_pin, initial_value=False)
        # 1 extra pulse selects channel A at gain 128 (HX711 default/typical
        # load-cell wiring); see HX711 datasheet timing diagram.
        self._extra_pulses = gain_pulses

    def is_ready(self) -> bool:
        return not self._dt.value

    def read_raw(self) -> int:
        if not self.is_ready():
            raise TimeoutError("HX711 not ready (DT pin still high)")
        value = 0
        for _ in range(24):
            self._sck.on()
            time.sleep(HX711_SETTLE_SEC)
            self._sck.off()
            time.sleep(HX711_SETTLE_SEC)
            value = (value << 1) | (1 if self._dt.value else 0)
        for _ in range(self._extra_pulses):
            self._sck.on()
            time.sleep(HX711_SETTLE_SEC)
            self._sck.off()
            time.sleep(HX711_SETTLE_SEC)
        if value & 0x800000:  # sign-extend the 24-bit two's complement value
            value -= 1 << 24
        return value

    def close(self) -> None:
        self._dt.close()
        self._sck.close()


class LoadcellGuard(Node):
    def __init__(self) -> None:
        super().__init__("loadcell_guard")
        self.declare_parameter("dt_pin", 5)
        self.declare_parameter("sck_pin", 6)
        self.declare_parameter("tare_offset", 0)
        self.declare_parameter("grams_per_count", 0.0)
        self.declare_parameter("low_threshold_g", 0.0)
        self.declare_parameter("high_threshold_g", 100000.0)
        self.declare_parameter("poll_hz", 5.0)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("stale_after_sec", 2.0)

        weight_qos = QoSProfile(depth=1)
        weight_qos.reliability = ReliabilityPolicy.RELIABLE
        weight_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.weight_publisher = self.create_publisher(
            Float32, "/loadcell/weight_grams", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/loadcell_guard/status", weight_qos
        )
        self.overweight_publisher = self.create_publisher(
            Bool, "/loadcell_guard/above_threshold", 10
        )
        self.underweight_publisher = self.create_publisher(
            Bool, "/loadcell_guard/below_threshold", 10
        )
        self.create_service(Trigger, "/loadcell_guard/tare", self._tare_service)

        self._tare_offset = int(self.get_parameter("tare_offset").value)
        self._recent: list[float] = []
        self._last_grams: Optional[float] = None
        self._last_read_at = 0.0
        self._hx711: Optional[HX711] = None
        self._init_error: Optional[str] = None
        try:
            self._hx711 = HX711(
                dt_pin=int(self.get_parameter("dt_pin").value),
                sck_pin=int(self.get_parameter("sck_pin").value),
            )
        except Exception as error:  # gpiozero/lgpio failures vary by platform
            self._init_error = str(error)
            self.get_logger().error(f"HX711 초기화 실패, 로드셀 비활성: {error}")

        frequency = max(0.5, float(self.get_parameter("poll_hz").value))
        self.create_timer(1.0 / frequency, self._poll)
        self.get_logger().info("loadcell_guard 시작됨")

    def _tare_service(self, request, response):
        if self._hx711 is None:
            response.success = False
            response.message = self._init_error or "HX711 unavailable"
            return response
        try:
            self._tare_offset = self._hx711.read_raw()
            response.success = True
            response.message = f"tare_offset={self._tare_offset}"
            self.get_logger().info(f"영점 재조정: tare_offset={self._tare_offset}")
        except (TimeoutError, OSError) as error:
            response.success = False
            response.message = str(error)
        return response

    def _poll(self) -> None:
        if self._hx711 is None:
            self._publish(None)
            return
        try:
            raw = self._hx711.read_raw()
        except (TimeoutError, OSError) as error:
            self.get_logger().warn(f"로드셀 읽기 실패: {error}", throttle_duration_sec=5.0)
            self._publish(None)
            return
        grams_per_count = float(self.get_parameter("grams_per_count").value)
        if grams_per_count == 0.0:
            self.get_logger().warn(
                "grams_per_count 캘리브레이션이 안 돼 있음 (0.0) -- "
                "ROBOT_LOADCELL_GRAMS_PER_COUNT 설정 필요",
                throttle_duration_sec=30.0,
            )
            self._publish(None)
            return
        grams = raw_to_grams(raw, self._tare_offset, grams_per_count)

        window = max(1, int(self.get_parameter("smoothing_window").value))
        self._recent.append(grams)
        del self._recent[:-window]
        smoothed = sum(self._recent) / len(self._recent)

        self._last_grams = smoothed
        self._last_read_at = time.monotonic()
        self._publish(smoothed)

    def _publish(self, grams: Optional[float]) -> None:
        low = float(self.get_parameter("low_threshold_g").value)
        high = float(self.get_parameter("high_threshold_g").value)
        age = (
            time.monotonic() - self._last_read_at
            if self._last_read_at > 0.0
            else float("inf")
        )
        stale_after = float(self.get_parameter("stale_after_sec").value)
        effective_grams = None if age > stale_after else grams
        state = classify_weight(effective_grams, low, high)

        if effective_grams is not None:
            weight_msg = Float32()
            weight_msg.data = float(effective_grams)
            self.weight_publisher.publish(weight_msg)

        above = Bool()
        above.data = state == "above"
        self.overweight_publisher.publish(above)
        below = Bool()
        below.data = state == "below"
        self.underweight_publisher.publish(below)

        status = String()
        status.data = json.dumps(
            {
                "connected": self._hx711 is not None,
                "grams": effective_grams,
                "state": state,
                "low_threshold_g": low,
                "high_threshold_g": high,
                "age_s": None if age == float("inf") else round(age, 3),
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)

    def destroy_node(self):
        if self._hx711 is not None:
            self._hx711.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LoadcellGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
