#!/usr/bin/env python3
"""Clear Nav2 costmaps and request a plan without commanding robot motion."""

import argparse
import math
import sys

import rclpy
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node


class NavigationProbe(Node):
    def __init__(self) -> None:
        super().__init__("navigation_probe")
        self._planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def clear_costmap(self, service_name: str) -> bool:
        client = self.create_client(ClearEntireCostmap, service_name)
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"service unavailable: {service_name}")
            return False

        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if not future.done() or future.exception() is not None:
            self.get_logger().error(
                f"costmap clear failed: {service_name}; exception={future.exception()}"
            )
            return False

        self.get_logger().info(f"cleared: {service_name}")
        return True

    def compute_path(self, x: float, y: float, yaw: float) -> int:
        if not self._planner.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("planner action server unavailable")
            return 2

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.planner_id = "GridBased"
        goal.use_start = False

        send_future = self._planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        if not send_future.done() or send_future.exception() is not None:
            self.get_logger().error("planner goal request timed out")
            return 3

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("planner rejected goal")
            return 4

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done() or result_future.exception() is not None:
            self.get_logger().error("planner result timed out")
            return 5

        wrapped_result = result_future.result()
        poses = wrapped_result.result.path.poses
        self.get_logger().info(
            f"planner status={wrapped_result.status}, path poses={len(poses)}, "
            f"goal=({x:.2f}, {y:.2f}, {yaw:.2f})"
        )
        if not poses:
            self.get_logger().error("planner returned an empty path")
            return 6

        first = poses[0].pose.position
        last = poses[-1].pose.position
        self.get_logger().info(
            f"non-empty path: ({first.x:.2f}, {first.y:.2f}) -> "
            f"({last.x:.2f}, {last.y:.2f})"
        )
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--skip-clear", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = NavigationProbe()
    try:
        services = (
            "/global_costmap/clear_entirely_global_costmap",
            "/local_costmap/clear_entirely_local_costmap",
        )
        if not args.skip_clear and not all(node.clear_costmap(name) for name in services):
            return 1
        return node.compute_path(args.x, args.y, args.yaw)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
