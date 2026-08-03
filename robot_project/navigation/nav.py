#!/usr/bin/env python3
"""
Navigation 노드 (nav.py) - STEP 3용 뼈대 코드
---------------------------------------------
SLAM으로 만든 지도(map.yaml)를 Nav2에 올린 뒤,
목표 좌표(goal pose)를 Nav2 액션 서버로 보내서 로봇이 스스로 이동하게 하는 부분.

주의: 이 노드를 쓰기 전에 아래가 먼저 준비되어 있어야 함
  1) SLAM으로 만든 map.yaml / map.pgm (../slam 폴더)
  2) nav2_bringup 으로 map_server + amcl + planner 등이 실행 중이어야 함

우선순위가 낮은 STEP이므로 지금은 최소 골격만 작성해둠.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavGoalNode(Node):
    def __init__(self):
        super().__init__('nav_goal_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        # TODO: yaw 값을 실제로 쓰려면 quaternion 변환 추가 필요 (tf_transformations 등)

        self._client.wait_for_server()
        self.get_logger().info(f'목표 지점 전송: x={x}, y={y}')
        self._client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalNode()
    # 예시: 테스트용 임시 좌표. 지도 생성 후 실제 좌표로 교체 필요
    node.send_goal(1.0, 0.5)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
