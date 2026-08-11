#!/usr/bin/env python3
"""Summarize live scan and odometry quality without commanding the robot."""

from __future__ import annotations

import json
import math
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MappingTopicDiagnostics(Node):
    def __init__(self, duration: float) -> None:
        super().__init__("mapping_topic_diagnostics")
        self.deadline = time.monotonic() + duration
        self.scans: dict[str, list[dict[str, object]]] = {
            topic: [] for topic in ("/scan", "/scan_fixed", "/scan_slam")
        }
        self.odometry: dict[str, list[dict[str, float]]] = {
            topic: [] for topic in ("/odom_raw", "/odom_nav")
        }
        for topic in self.scans:
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, name=topic: self._on_scan(name, message),
                10,
            )
        for topic in self.odometry:
            self.create_subscription(
                Odometry,
                topic,
                lambda message, name=topic: self._on_odom(name, message),
                10,
            )

    def _on_scan(self, topic: str, message: LaserScan) -> None:
        finite = [float(value) for value in message.ranges if math.isfinite(value)]
        front = [
            float(value)
            for index, value in enumerate(message.ranges)
            if math.isfinite(value)
            and abs(message.angle_min + index * message.angle_increment)
            <= math.radians(30.0)
        ]
        self.scans[topic].append(
            {
                "stamp": message.header.stamp.sec
                + message.header.stamp.nanosec / 1_000_000_000.0,
                "frame": message.header.frame_id,
                "beams": len(message.ranges),
                "finite": len(finite),
                "minimum": min(finite, default=None),
                "median": statistics.median(finite) if finite else None,
                "maximum": max(finite, default=None),
                "front_minimum": min(front, default=None),
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "angle_increment": float(message.angle_increment),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
            }
        )

    def _on_odom(self, topic: str, message: Odometry) -> None:
        pose = message.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z),
            1.0 - 2.0 * pose.orientation.z * pose.orientation.z,
        )
        self.odometry[topic].append(
            {
                "stamp": message.header.stamp.sec
                + message.header.stamp.nanosec / 1_000_000_000.0,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw,
                "linear_x": float(message.twist.twist.linear.x),
                "angular_z": float(message.twist.twist.angular.z),
            }
        )

    def result(self) -> dict[str, object]:
        result: dict[str, object] = {"scans": {}, "odometry": {}}
        for topic, frames in self.scans.items():
            if not frames:
                result["scans"][topic] = {"frames": 0}
                continue
            finite_counts = [int(frame["finite"]) for frame in frames]
            stamps = [float(frame["stamp"]) for frame in frames]
            elapsed = max(stamps) - min(stamps)
            exemplar = frames[-1]
            result["scans"][topic] = {
                "frames": len(frames),
                "rate_hz": (len(frames) - 1) / elapsed if elapsed > 0.0 else 0.0,
                "frame": exemplar["frame"],
                "beams": exemplar["beams"],
                "finite_mean": statistics.mean(finite_counts),
                "finite_min": min(finite_counts),
                "finite_max": max(finite_counts),
                "minimum_m": exemplar["minimum"],
                "median_m": exemplar["median"],
                "maximum_m": exemplar["maximum"],
                "front_minimum_m": exemplar["front_minimum"],
                "angle_min": exemplar["angle_min"],
                "angle_max": exemplar["angle_max"],
                "angle_increment": exemplar["angle_increment"],
                "range_min": exemplar["range_min"],
                "range_max": exemplar["range_max"],
            }
        for topic, samples in self.odometry.items():
            if not samples:
                result["odometry"][topic] = {"samples": 0}
                continue
            first = samples[0]
            last = samples[-1]
            elapsed = last["stamp"] - first["stamp"]
            result["odometry"][topic] = {
                "samples": len(samples),
                "rate_hz": (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0,
                "start": {key: first[key] for key in ("x", "y", "yaw")},
                "end": {key: last[key] for key in ("x", "y", "yaw")},
                "delta": {
                    "x": last["x"] - first["x"],
                    "y": last["y"] - first["y"],
                    "yaw": math.atan2(
                        math.sin(last["yaw"] - first["yaw"]),
                        math.cos(last["yaw"] - first["yaw"]),
                    ),
                },
                "last_twist": {
                    "linear_x": last["linear_x"],
                    "angular_z": last["angular_z"],
                },
            }
        return result


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = MappingTopicDiagnostics(duration)
    try:
        while rclpy.ok() and time.monotonic() < node.deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        print(json.dumps(node.result(), indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
