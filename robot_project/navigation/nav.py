#!/usr/bin/env python3
"""
Navigation 노드 (nav.py)
----------------------------
SLAM으로 만든 지도(map.yaml)를 Nav2에 올린 뒤,
목표 좌표(goal pose)를 Nav2 액션 서버로 보내서 로봇이 스스로 이동하게 하는 부분.

주의: 이 노드를 쓰기 전에 아래가 먼저 준비되어 있어야 함
  1) SLAM으로 만든 map.yaml / map.pgm (../slam 폴더)
  2) nav2_bringup 으로 map_server + amcl + planner 등이 실행 중이어야 함

목표 x/y/yaw를 ROS 파라미터로 받고, GUI 중지 시 Nav2 목표 취소를 지원한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavGoalNode(Node):
    def __init__(self):
        super().__init__('nav_goal_node')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('server_timeout', 10.0)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._goal_future = None
        self._cancel_requested = False

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        timeout = float(self.get_parameter('server_timeout').value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f'Nav2 navigate_to_pose 서버를 {timeout:.1f}초 안에 찾지 못했습니다'
            )
            return False
        self.get_logger().info(f'목표 지점 전송: x={x}, y={y}, yaw={yaw}')
        self._goal_future = self._client.send_goal_async(goal_msg)
        self._goal_future.add_done_callback(self._goal_response)
        return True

    def _goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'목표 전송 실패: {error}')
            return
        if not goal_handle.accepted:
            self.get_logger().error('Nav2가 목표를 거부했습니다')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Nav2가 목표를 수락했습니다')
        if self._cancel_requested:
            self.cancel_goal()

    def cancel_goal(self) -> None:
        """Request cancellation of the active Nav2 goal, if one exists."""

        self._cancel_requested = True
        if self._goal_handle is None:
            self.get_logger().info('목표 수락 대기 중: 수락 즉시 취소합니다')
            return
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda _: self.get_logger().info('Navigation 목표 취소 요청을 전송했습니다')
        )

    def send_configured_goal(self) -> bool:
        """Send the goal supplied through ROS parameters."""

        return self.send_goal(
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_yaw').value),
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalNode()
    node.send_configured_goal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cancel_goal()
            rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
