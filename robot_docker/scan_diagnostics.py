#!/usr/bin/env python3
"""Print close LaserScan returns with their indices and physical angles."""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanDiagnostics(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("scan_diagnostics")
        self.message = None
        self.subscription = self.create_subscription(LaserScan, topic, self.receive, 10)

    def receive(self, message: LaserScan) -> None:
        self.message = message


def contiguous_runs(indices: list[int]) -> list[tuple[int, int]]:
    if not indices:
        return []
    runs = []
    start = previous = indices[0]
    for index in indices[1:]:
        if index != previous + 1:
            runs.append((start, previous))
            start = index
        previous = index
    runs.append((start, previous))
    return runs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/scan_fixed")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ScanDiagnostics(args.topic)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while node.message is None and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.message is None:
            print(f"no scan received from {args.topic}")
            return 1

        scan = node.message
        valid = []
        for index, distance in enumerate(scan.ranges):
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                angle = scan.angle_min + index * scan.angle_increment
                valid.append((distance, index, math.degrees(angle)))

        valid.sort()
        print(
            f"topic={args.topic} frame={scan.header.frame_id} count={len(scan.ranges)} "
            f"angle_min={math.degrees(scan.angle_min):.1f}deg "
            f"angle_max={math.degrees(scan.angle_max):.1f}deg "
            f"increment={math.degrees(scan.angle_increment):.3f}deg "
            f"range=[{scan.range_min:.3f}, {scan.range_max:.3f}]"
        )
        for threshold in (0.22, 0.30, 0.50, 1.00):
            close = [item for item in valid if item[0] <= threshold]
            print(f"valid <= {threshold:.2f}m: {len(close)}")
            if threshold <= 0.30:
                indices = sorted(item[1] for item in close)
                runs = contiguous_runs(indices)
                run_text = ", ".join(
                    f"{start}-{end} "
                    f"({math.degrees(scan.angle_min + start * scan.angle_increment):.0f}.."
                    f"{math.degrees(scan.angle_min + end * scan.angle_increment):.0f}deg)"
                    for start, end in runs
                )
                print(f"  runs: {run_text or 'none'}")
        print("20 nearest valid returns (range, index, angle):")
        for distance, index, angle in valid[:20]:
            print(f"  {distance:.3f}m  index={index:3d}  angle={angle:7.2f}deg")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
