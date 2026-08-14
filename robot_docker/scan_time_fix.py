#!/usr/bin/env python3
"""Restamp and filter laser scans for navigation or SLAM.

Navigation gets a lightly cleaned scan so a thin, real obstacle is never
hidden.  A second instance can enable spatial and temporal filtering for SLAM,
where rejecting one-frame speckles is more important than minimum latency.
"""

import math
import statistics
from collections import deque
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


SELF_REFLECTION_MIN_ANGLE_DEG = -155.0
SELF_REFLECTION_MAX_ANGLE_DEG = -145.0
SELF_REFLECTION_MAX_RANGE_M = 0.22
REAR_HOUSING_MIN_ANGLE_DEG = -172.0
REAR_HOUSING_MAX_ANGLE_DEG = -164.0
REAR_HOUSING_MAX_RANGE_M = 0.16


def delayed_stamp_parts(
    now_nanoseconds: int, delay_seconds: float
) -> tuple[int, int]:
    """Return a non-negative ROS stamp delayed from the local receive time.

    The scan and wheel-odometry callbacks run independently.  Stamping a scan
    at its exact receive time commonly puts it just ahead of the newest odom
    transform, so slam_toolbox/Nav2 discard it before the following odom sample
    arrives.  A small delay places the scan between transforms already held in
    the tf2 cache and allows interpolation without changing the measured range.
    """

    bounded_delay = max(0.0, min(float(delay_seconds), 2.0))
    stamp_nanoseconds = max(
        0, int(now_nanoseconds) - int(bounded_delay * 1_000_000_000)
    )
    return divmod(stamp_nanoseconds, 1_000_000_000)


def synchronized_stamp_parts(
    now_nanoseconds: int,
    odometry_stamp_nanoseconds: int,
    odometry_received_nanoseconds: int,
    maximum_odometry_age: float,
    fallback_delay_seconds: float,
) -> tuple[int, int]:
    """Use a recent odometry timestamp known to have a matching TF sample.

    Restamping scans and wheel odometry from two independent callbacks can put
    a scan just outside the transform history seen by slam_toolbox.  Aligning
    the scan to the latest relayed odometry sample gives tf2 an exact transform
    instead of relying on callback timing or extrapolation.
    """

    age_nanoseconds = int(now_nanoseconds) - int(odometry_received_nanoseconds)
    maximum_age_nanoseconds = int(max(0.0, maximum_odometry_age) * 1_000_000_000)
    if (
        odometry_stamp_nanoseconds > 0
        and odometry_received_nanoseconds > 0
        and 0 <= age_nanoseconds <= maximum_age_nanoseconds
    ):
        return divmod(int(odometry_stamp_nanoseconds), 1_000_000_000)
    return delayed_stamp_parts(now_nanoseconds, fallback_delay_seconds)


def is_self_reflection(angle_radians: float, distance: float) -> bool:
    angle_degrees = math.degrees(angle_radians)
    return math.isfinite(distance) and (
        (
            SELF_REFLECTION_MIN_ANGLE_DEG
            <= angle_degrees
            <= SELF_REFLECTION_MAX_ANGLE_DEG
            and distance <= SELF_REFLECTION_MAX_RANGE_M
        )
        or (
            REAR_HOUSING_MIN_ANGLE_DEG
            <= angle_degrees
            <= REAR_HOUSING_MAX_ANGLE_DEG
            and distance <= REAR_HOUSING_MAX_RANGE_M
        )
    )


def sanitize_ranges(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    maximum_range_m: float = 0.0,
) -> list[float]:
    """Reject invalid ranges and the measured rear chassis reflection."""

    cleaned: list[float] = []
    for index, raw_distance in enumerate(ranges):
        distance = float(raw_distance)
        angle = angle_min + index * angle_increment
        effective_maximum = (
            min(range_max, maximum_range_m) if maximum_range_m > 0.0 else range_max
        )
        if (
            not math.isfinite(distance)
            or distance < range_min
            or distance > effective_maximum
            or is_self_reflection(angle, distance)
        ):
            cleaned.append(math.inf)
        else:
            cleaned.append(distance)
    return cleaned


def remove_spatial_speckles(
    ranges: Sequence[float], *, radius: int = 2, tolerance: float = 0.12
) -> list[float]:
    """Remove isolated finite returns that have no nearby supporting beam.

    This is deliberately optional and is enabled only on the SLAM stream. A
    post/tree normally occupies adjacent beams, while the observed LiDAR noise
    appears as a single unsupported point.
    """

    if radius <= 0:
        return list(ranges)
    filtered = list(ranges)
    count = len(ranges)
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        start = max(0, index - radius)
        end = min(count, index + radius + 1)
        supported = False
        allowed_delta = max(tolerance, distance * 0.04)
        for neighbor_index in range(start, end):
            if neighbor_index == index:
                continue
            neighbor = ranges[neighbor_index]
            if math.isfinite(neighbor) and abs(neighbor - distance) <= allowed_delta:
                supported = True
                break
        if not supported:
            filtered[index] = math.inf
    return filtered


class TemporalMedianFilter:
    """Keep ranges seen in multiple recent scans and publish their median."""

    def __init__(
        self, window: int = 1, minimum_hits: int = 1, tolerance: float = 0.15
    ) -> None:
        self.window = max(1, int(window))
        self.minimum_hits = max(1, min(int(minimum_hits), self.window))
        self.tolerance = max(0.0, float(tolerance))
        self.history: deque[list[float]] = deque(maxlen=self.window)

    @property
    def ready(self) -> bool:
        """Whether enough frames exist to publish a meaningful first scan."""

        return len(self.history) >= self.minimum_hits

    def update(self, ranges: Sequence[float]) -> list[float]:
        current = list(ranges)
        if self.history and len(self.history[-1]) != len(current):
            self.history.clear()
        self.history.append(current)
        if self.window == 1:
            return current

        output: list[float] = []
        for index in range(len(current)):
            finite = [
                scan[index] for scan in self.history if math.isfinite(scan[index])
            ]
            clusters = [
                [
                    value
                    for value in finite
                    if abs(value - center)
                    <= max(self.tolerance, abs(center) * 0.04)
                ]
                for center in finite
            ]
            consistent = max(clusters, key=len, default=[])
            if len(consistent) < self.minimum_hits:
                output.append(math.inf)
            else:
                output.append(float(statistics.median(consistent)))
        return output


class ScanTimeFixNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_scan_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_fixed")
        self.declare_parameter("max_publish_hz", 0.0)
        self.declare_parameter("timestamp_delay_seconds", 0.0)
        self.declare_parameter("odometry_topic", "/odom_nav")
        self.declare_parameter("maximum_odometry_age", 0.5)
        self.declare_parameter("spatial_filter_radius", 0)
        self.declare_parameter("spatial_tolerance", 0.12)
        self.declare_parameter("temporal_window", 1)
        self.declare_parameter("temporal_minimum_hits", 1)
        self.declare_parameter("temporal_tolerance", 0.15)
        self.declare_parameter("maximum_range_m", 0.0)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.max_publish_hz = float(self.get_parameter("max_publish_hz").value)
        self.timestamp_delay_seconds = max(
            0.0,
            min(
                float(self.get_parameter("timestamp_delay_seconds").value),
                2.0,
            ),
        )
        self.odometry_topic = str(self.get_parameter("odometry_topic").value)
        self.maximum_odometry_age = max(
            0.0, float(self.get_parameter("maximum_odometry_age").value)
        )
        self.spatial_filter_radius = int(
            self.get_parameter("spatial_filter_radius").value
        )
        self.spatial_tolerance = float(
            self.get_parameter("spatial_tolerance").value
        )
        self.temporal_filter = TemporalMedianFilter(
            int(self.get_parameter("temporal_window").value),
            int(self.get_parameter("temporal_minimum_hits").value),
            float(self.get_parameter("temporal_tolerance").value),
        )
        self.maximum_range_m = float(self.get_parameter("maximum_range_m").value)
        self._last_publish_nanoseconds = 0
        self._latest_odometry_stamp_nanoseconds = 0
        self._latest_odometry_received_nanoseconds = 0
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan, self.input_topic, self.callback, 10
        )
        self.odometry_subscription = self.create_subscription(
            Odometry, self.odometry_topic, self.odometry_callback, 10
        )
        self.get_logger().info(
            f"navigation scan filter: {self.input_topic} -> {self.output_topic}; "
            f"self-reflection={SELF_REFLECTION_MIN_ANGLE_DEG:.0f}.."
            f"{SELF_REFLECTION_MAX_ANGLE_DEG:.0f} deg <= "
            f"{SELF_REFLECTION_MAX_RANGE_M:.2f} m; "
            f"max_publish_hz={self.max_publish_hz:.1f}; "
            f"timestamp_delay={self.timestamp_delay_seconds:.3f} s; "
            f"timestamp_source={self.odometry_topic} "
            f"(max_age={self.maximum_odometry_age:.2f} s); "
            f"spatial_radius={self.spatial_filter_radius}; "
            f"temporal={self.temporal_filter.minimum_hits}/"
            f"{self.temporal_filter.window}; maximum_range_m="
            f"{self.maximum_range_m:.2f}"
        )

    def odometry_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp
        stamp_nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_nanoseconds <= 0:
            return
        self._latest_odometry_stamp_nanoseconds = stamp_nanoseconds
        self._latest_odometry_received_nanoseconds = self.get_clock().now().nanoseconds

    def callback(self, message: LaserScan) -> None:
        now = self.get_clock().now()
        if self.max_publish_hz > 0.0 and self._last_publish_nanoseconds:
            minimum_period = int(1_000_000_000 / self.max_publish_hz)
            if now.nanoseconds - self._last_publish_nanoseconds < minimum_period:
                return
        stamp_sec, stamp_nanosec = synchronized_stamp_parts(
            now.nanoseconds,
            self._latest_odometry_stamp_nanoseconds,
            self._latest_odometry_received_nanoseconds,
            self.maximum_odometry_age,
            self.timestamp_delay_seconds,
        )
        message.header.stamp.sec = stamp_sec
        message.header.stamp.nanosec = stamp_nanosec
        ranges = sanitize_ranges(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=float(message.range_max),
            maximum_range_m=self.maximum_range_m,
        )
        ranges = remove_spatial_speckles(
            ranges,
            radius=self.spatial_filter_radius,
            tolerance=self.spatial_tolerance,
        )
        filtered = self.temporal_filter.update(ranges)
        # Do not let slam_toolbox initialize from an all-inf warm-up scan. With
        # the robot stationary, its travel threshold would otherwise preserve
        # that empty 0x0 map until physical movement begins.
        if not self.temporal_filter.ready:
            return
        self._last_publish_nanoseconds = now.nanoseconds
        message.ranges = filtered
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
