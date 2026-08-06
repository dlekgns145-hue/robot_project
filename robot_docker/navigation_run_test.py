#!/usr/bin/env python3
"""Send one bounded Nav2 goal and cancel it if it does not finish in time."""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class NavigationRunTest(Node):
    def __init__(self) -> None:
        super().__init__("navigation_run_test")
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._command_mode = self.create_client(
            SetBool, "/cmd_bridge/navigation_mode"
        )
        self._emergency_subscription = self.create_subscription(
            Bool, "/cmd_bridge/emergency_stop", self._on_emergency_stop, 10
        )
        self._last_reported_distance = None
        self._goal_handle = None
        self._emergency_stop_received = False

    def _set_navigation_mode(self, enabled: bool) -> bool:
        if not self._command_mode.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("command bridge navigation lock unavailable")
            return False
        request = SetBool.Request()
        request.data = enabled
        future = self._command_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.exception() is not None:
            self.get_logger().error("command bridge navigation lock timed out")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(
                f"command bridge navigation lock rejected: {response.message}"
            )
            return False
        self.get_logger().info(response.message)
        return True

    def _on_emergency_stop(self, message: Bool) -> None:
        if not message.data:
            return
        self._emergency_stop_received = True
        self.get_logger().error("emergency stop received; cancelling navigation")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def feedback(self, message) -> None:
        distance = message.feedback.distance_remaining
        if (
            self._last_reported_distance is None
            or abs(distance - self._last_reported_distance) >= 0.1
        ):
            self.get_logger().info(f"distance remaining: {distance:.2f} m")
            self._last_reported_distance = distance

    def run(self, x: float, y: float, yaw: float, timeout: float) -> int:
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("navigate_to_pose action server unavailable")
            return 2
        if not self._set_navigation_mode(True):
            return 8

        try:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_future = self._client.send_goal_async(
                goal, feedback_callback=self.feedback
            )
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
            if not send_future.done() or send_future.exception() is not None:
                self.get_logger().error("goal request timed out")
                return 3

            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("navigation goal rejected")
                return 4
            self._goal_handle = goal_handle

            self.get_logger().info(
                f"goal accepted: ({x:.2f}, {y:.2f}, {yaw:.2f}), "
                f"timeout={timeout:.0f}s"
            )
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            if not result_future.done():
                self.get_logger().error("navigation timed out; cancelling goal")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self, cancel_future, timeout_sec=10.0
                )
                return 5
            if result_future.exception() is not None:
                self.get_logger().error(
                    f"navigation exception: {result_future.exception()}"
                )
                return 6

            status = result_future.result().status
            self.get_logger().info(f"navigation finished with status={status}")
            if self._emergency_stop_received:
                return 9
            if status != GoalStatus.STATUS_SUCCEEDED:
                return 7
            return 0
        finally:
            self._goal_handle = None
            self._set_navigation_mode(False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavigationRunTest()
    try:
        return node.run(args.x, args.y, args.yaw, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
