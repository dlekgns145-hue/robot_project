#!/usr/bin/env python3
"""
사람 따라가기 노드 (follow_person.py)
------------------------------------
/person_detection 토픽(YOLO 결과)을 받아서
사람의 좌우 위치와 거리(박스 크기)를 계산하고, 그에 맞는 /cmd_vel 을 발행한다.

/follow_person/start, /follow_person/stop (std_srvs/Trigger) 서비스로 켜고 끌 수 있다.
기본값은 꺼짐(enabled=False) -- mapping-runtime과 같은 이유로, 아무도 명시적으로
시작시키지 않았는데 로봇이 사람을 쫓아다니기 시작하면 안 되기 때문.

로직 (STEP 6~8 그대로 구현):
  - 중심 x좌표가 왼쪽  -> 좌회전
  - 중심 x좌표가 오른쪽 -> 우회전
  - 중심 x좌표가 가운데 -> 회전 없음(직진 유지)
  - 박스가 크면(가까우면) -> 정지
  - 박스가 작으면(멀면)   -> 전진
  - 일정 시간 사람이 안 보이면 -> 안전 정지
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class FollowPersonNode(Node):
    def __init__(self):
        super().__init__('follow_person_node')

        # ---- 파라미터 (STEP 1: 속도 조절은 여기서 진행) ----
        self.declare_parameter('linear_speed', 0.2)             # 전진 속도. 0.2 -> 0.5 등으로 바꿔가며 테스트
        self.declare_parameter('angular_speed', 0.4)             # 회전 속도
        self.declare_parameter('center_tolerance_ratio', 0.15)   # 이 비율 안쪽이면 '가운데'로 간주 (프레임폭 기준)
        self.declare_parameter('stop_box_height_ratio', 0.6)     # 박스 높이가 프레임의 이 비율 이상이면 '가까움'
        self.declare_parameter('lost_person_timeout', 1.0)       # 사람 놓친 뒤 정지까지 대기시간(초)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_tol_ratio = self.get_parameter('center_tolerance_ratio').value
        self.stop_height_ratio = self.get_parameter('stop_box_height_ratio').value
        self.lost_timeout = self.get_parameter('lost_person_timeout').value

        self.enabled = False
        self.last_seen_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Float32MultiArray, '/person_detection', self.detection_callback, 10)
        # Publish to /cmd_vel_follow, not /cmd_vel directly. robot_cmd_bridge.py
        # is the single-point owner of /cmd_vel (it warns on publisher contention
        # if anything else writes there). /cmd_vel_server exists but is gated to
        # navigation_mode (Nav2/mapping only) -- a follow command sent there would
        # be silently dropped since navigation_mode is off while following.
        # robot_cmd_bridge.py relays /cmd_vel_follow through the same publish_cmd()
        # path GUI manual driving uses, so follow-mode inherits the existing LiDAR
        # obstacle-avoidance state machine and command-timeout safety stop.
        self.pub = self.create_publisher(Twist, '/cmd_vel_follow', 10)

        self.create_service(Trigger, '/follow_person/start', self._handle_start)
        self.create_service(Trigger, '/follow_person/stop', self._handle_stop)

        # 사람을 잠깐이라도 놓쳤을 때 안전하게 정지시키기 위한 타이머 (0.2초마다 체크)
        self.safety_timer = self.create_timer(0.2, self.safety_check)

        self.get_logger().info(
            f'Follow Person Node 시작됨 (linear_speed={self.linear_speed}, angular_speed={self.angular_speed}, '
            f'enabled={self.enabled})'
        )

    def _handle_start(self, request, response):
        self.enabled = True
        self.last_seen_time = self.get_clock().now()
        self.get_logger().info('사람 따라가기 시작')
        response.success = True
        response.message = 'follow started'
        return response

    def _handle_stop(self, request, response):
        self.enabled = False
        self.pub.publish(Twist())  # 즉시 정지
        self.get_logger().info('사람 따라가기 정지')
        response.success = True
        response.message = 'follow stopped'
        return response

    def detection_callback(self, msg: Float32MultiArray):
        if not self.enabled:
            return

        found, cx, cy, bw, bh, frame_w, frame_h = msg.data

        if found < 0.5:
            # 이번 프레임엔 사람 없음 -> safety_check가 알아서 정지 처리
            return

        self.last_seen_time = self.get_clock().now()

        twist = Twist()
        center_x = frame_w / 2.0
        tolerance = frame_w * self.center_tol_ratio

        # ---- STEP 6: 좌/우/중앙 판단 ----
        if cx < center_x - tolerance:
            twist.angular.z = self.angular_speed        # 좌회전
        elif cx > center_x + tolerance:
            twist.angular.z = -self.angular_speed       # 우회전
        else:
            twist.angular.z = 0.0                       # 가운데

        # ---- STEP 7~8: 박스 크기(거리)로 전진/정지 판단 ----
        height_ratio = bh / frame_h if frame_h > 0 else 0.0
        if height_ratio >= self.stop_height_ratio:
            twist.linear.x = 0.0                        # 가까움 -> 정지
        else:
            twist.linear.x = self.linear_speed          # 멀음 -> 전진

        self.pub.publish(twist)

    def safety_check(self):
        """사람을 lost_timeout 초 이상 못 보면 무조건 정지시킨다."""
        if not self.enabled:
            return
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if elapsed > self.lost_timeout:
            self.pub.publish(Twist())  # 전부 0 -> 정지


def main(args=None):
    rclpy.init(args=args)
    node = FollowPersonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
