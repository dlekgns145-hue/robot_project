#!/usr/bin/env python3
"""Restamp the laser scan and remove the measured chassis reflection."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


SELF_REFLECTION_MIN_ANGLE_DEG = -155.0
SELF_REFLECTION_MAX_ANGLE_DEG = -145.0
SELF_REFLECTION_MAX_RANGE_M = 0.22


def is_self_reflection(angle_radians: float, distance: float) -> bool:
    angle_degrees = math.degrees(angle_radians)
    return (
        SELF_REFLECTION_MIN_ANGLE_DEG
        <= angle_degrees
        <= SELF_REFLECTION_MAX_ANGLE_DEG
        and math.isfinite(distance)
        and distance <= SELF_REFLECTION_MAX_RANGE_M
    )


class ScanTimeFixNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_scan_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_fixed")
        self.declare_parameter("max_publish_hz", 0.0)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.max_publish_hz = float(self.get_parameter("max_publish_hz").value)
        self._last_publish_nanoseconds = 0
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, self.input_topic, self.callback, 10
        )
        self.get_logger().info(
            f"navigation scan filter: {self.input_topic} -> {self.output_topic}; "
            f"self-reflection={SELF_REFLECTION_MIN_ANGLE_DEG:.0f}.."
            f"{SELF_REFLECTION_MAX_ANGLE_DEG:.0f} deg <= "
            f"{SELF_REFLECTION_MAX_RANGE_M:.2f} m; "
            f"max_publish_hz={self.max_publish_hz:.1f}"
        )

    def callback(self, message: LaserScan) -> None:
        now = self.get_clock().now()
        if self.max_publish_hz > 0.0 and self._last_publish_nanoseconds:
            minimum_period = int(1_000_000_000 / self.max_publish_hz)
            if now.nanoseconds - self._last_publish_nanoseconds < minimum_period:
                return
        self._last_publish_nanoseconds = now.nanoseconds
        message.header.stamp = now.to_msg()
        ranges = list(message.ranges)
        for index, distance in enumerate(ranges):
            angle = message.angle_min + index * message.angle_increment
            if distance < message.range_min or is_self_reflection(angle, distance):
                ranges[index] = math.inf
        message.ranges = ranges
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = ScanTimeFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
