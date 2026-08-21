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
  - 잠깐 놓치면(lost_person_timeout 이내) -> 마지막 속도 유지
  - 좀 더 오래 놓치면 -> 마지막으로 사람이 있던 방향으로 천천히 훑으며 탐색
    (탐색 중 다시 잡히는 사람이 원래 타겟과 같은 사람인지는 detect.py의
    얼굴/외형 재획득 로직이 이미 검증한다 -- found=1로 발행되는 순간부터는
    항상 "그 사람"이라고 신뢰해도 된다)
  - search_timeout까지 못 찾으면 -> 완전히 포기하고 정지 (무한 회전 방지)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class FollowPersonNode(Node):
    def __init__(self):
        super().__init__('follow_person_node')

        # ---- 파라미터 (STEP 1: 속도 조절은 여기서 진행) ----
        self.declare_parameter('linear_speed', 0.2)             # 전진 속도. 0.2 -> 0.5 등으로 바꿔가며 테스트
        self.declare_parameter('angular_speed', 0.4)             # 회전 속도
        self.declare_parameter('center_tolerance_ratio', 0.15)   # 이 비율 안쪽이면 '가운데'로 간주 (프레임폭 기준)
        # 박스 높이가 프레임의 이 비율 이상이면 '가까움' -> 전진 중단(독립
        # 정지 조건). 원래 0.6이었는데, 실기에서 사람이 카메라에서 눈대중
        # 1.5~2m는 떨어져 있는데도 이미 0.62를 찍는 걸 확인했다 (2026-08-20,
        # 광각 렌즈 왜곡 + 카메라 틸트 각도 때문에 박스 비율과 실제 거리의
        # 대응 관계가 안 맞는 것으로 보임). 그래서 이 값 자체로 정지를
        # 판단하는 건 사실상 꺼두고(거의 안 닿을 만큼 높게 잡음), 대신
        # 실제 정지거리 판단은 robot_cmd_bridge.py의 LiDAR 실측
        # (FOLLOW_HARD_STOP_DISTANCE_M / FOLLOW_TARGET_STOP_DISTANCE_M)에
        # 맡긴다. 이 노드가 계속 보내는 박스 비율(아래 height_ratio_pub)은
        # 거기서 "LiDAR가 가깝다는 게 실제로 이 사람인지" 확인하는 낮은
        # 기준(FOLLOW_CAMERA_CONFIRM_RATIO)으로만 쓰인다.
        self.declare_parameter('stop_box_height_ratio', 0.95)
        # 사람 놓친 뒤 탐색 시작까지 대기시간(초). 원래 1.0초였다가 "오다가
        # 멈추고" 스터터 때문에 2.0초로 늘렸는데, 그 스터터의 진짜 원인은
        # mapping-runtime이 CPU를 잡아먹던 것이었고(2026-08-20 별도로 확인,
        # 지금은 follow_start 시 자동 정지되게 고침) -- 2.0초는 그거에 비해
        # 과했다. 놓쳤을 때 반응이 너무 느리다는 피드백을 받아 1.2초로
        # 되돌린다: person-detect ~3.4Hz 기준 프레임 4개 정도는 여전히
        # 봐주면서, 진짜 놓친 경우엔 훨씬 빨리 반응한다.
        self.declare_parameter('lost_person_timeout', 1.2)
        # 탐색(둘러보기) 중 회전 속도. 예전엔 일반 추적(0.4)보다 훨씬 느리게
        # (0.25) 잡아뒀는데, 그러면 금방 다시 찾는 짧은 탐색 구간에서는 로봇이
        # 도는 게 거의 안 보여서 "그냥 멈춘 것"처럼 느껴졌다. 추적 속도에
        # 가깝게 올려서 탐색 중이라는 게 눈에 띄게 한다.
        self.declare_parameter('search_angular_speed', 0.35)
        self.declare_parameter('search_timeout', 12.0)           # 탐색 시작 후 완전히 포기하고 정지할 때까지(초)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_tol_ratio = self.get_parameter('center_tolerance_ratio').value
        self.stop_height_ratio = self.get_parameter('stop_box_height_ratio').value
        self.lost_timeout = self.get_parameter('lost_person_timeout').value
        self.search_angular_speed = self.get_parameter('search_angular_speed').value
        self.search_timeout = self.get_parameter('search_timeout').value

        self.enabled = False
        self.last_seen_time = self.get_clock().now()
        self.last_twist = Twist()
        # 사람이 마지막으로 향했던 회전 방향(+1=왼쪽/반시계, -1=오른쪽/시계).
        # 놓쳤을 때 이 방향부터 먼저 훑어야 다시 찾을 확률이 높다.
        self.last_turn_direction = 1.0
        self.searching = False

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
        # robot_cmd_bridge.py는 LiDAR로 정면거리를 정확하게 재지만, 그것만으로는
        # "가까이 있는 게 내가 따라가는 사람인지 그냥 가구인지" 구분을 못 한다
        # (가구 많은 공간에서 실제로 오인 사례 발생, 2026-08-20). 지금 보이는
        # 사람 박스가 실제로 얼마나 큰지(=화면에서 차지하는 비율)를 같이 보내서,
        # 브릿지가 "LiDAR도 가깝다 하고 카메라도 사람이 커 보인다고 할 때"만
        # 진짜 도착으로 판단하게 한다.
        self.height_ratio_pub = self.create_publisher(Float32, '/follow_person/box_height_ratio', 10)

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
            # 이번 프레임엔 사람 없음 -> safety_check가 알아서 정지 처리.
            # 카메라로 확인이 안 되는 상태이니 브릿지에도 그렇게 알려서,
            # 이 시점에 LiDAR가 뭘 가깝다고 재든 "확인된 사람"으로 오인해
            # 정지시키지 않게 한다.
            self.height_ratio_pub.publish(Float32(data=0.0))
            return

        self.last_seen_time = self.get_clock().now()
        self.searching = False

        twist = Twist()
        center_x = frame_w / 2.0
        tolerance = frame_w * self.center_tol_ratio

        # ---- STEP 6: 좌/우/중앙 판단 ----
        if cx < center_x - tolerance:
            twist.angular.z = self.angular_speed        # 좌회전
            self.last_turn_direction = 1.0
        elif cx > center_x + tolerance:
            twist.angular.z = -self.angular_speed       # 우회전
            self.last_turn_direction = -1.0
        else:
            twist.angular.z = 0.0                       # 가운데

        # ---- STEP 7~8: 박스 크기(거리)로 전진/정지 판단 ----
        height_ratio = bh / frame_h if frame_h > 0 else 0.0
        if height_ratio >= self.stop_height_ratio:
            twist.linear.x = 0.0                        # 가까움 -> 정지
        else:
            twist.linear.x = self.linear_speed          # 멀음 -> 전진

        self.last_twist = twist
        self.pub.publish(twist)
        self.height_ratio_pub.publish(Float32(data=float(height_ratio)))

    def safety_check(self):
        """0.2초마다 실행: 상황에 따라 추적 유지 / 탐색 / 정지 중 하나를 재전송한다.

        person-detect는 라즈베리파이 CPU에서 YOLO+ReID+얼굴인식을 전부 돌리느라
        초당 1프레임 안팎으로만 결과를 낸다 (실기 확인, 2026-08-20). 그런데
        robot_cmd_bridge.py는 명령이 0.5초 넘게 안 오면 자동으로 정지시킨다
        (수동 조종 중 연결이 끊겼을 때를 위한 안전장치). detection_callback만
        믿고 있으면 이 두 값 사이에서 감지 주기가 타임아웃보다 느려질 때마다
        매번 멈췄다 움직였다를 반복하게 된다. 그래서 감지 이벤트와 무관하게
        이 타이머가 0.2초마다 "마지막으로 계산한 속도"를 다시 보내 타임아웃
        안쪽으로 유지한다.

        사람을 놓친 지 lost_timeout을 넘기면 곧바로 정지하지 않고, 마지막으로
        사람이 향했던 방향부터 천천히 훑으며(search) 다시 찾는다. 같은 사람인지
        확인하는 건 detect.py의 얼굴/외형 재획득 로직이 이미 담당하므로, 여기서는
        "몸을 어느 쪽으로 돌릴지"만 결정한다. search_timeout까지도 못 찾으면
        완전히 포기하고 정지한다(무한 회전 방지).
        """
        if not self.enabled:
            return
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9

        if elapsed <= self.lost_timeout:
            self.searching = False
            self.pub.publish(self.last_twist)
            return

        search_elapsed = elapsed - self.lost_timeout
        if search_elapsed <= self.search_timeout:
            if not self.searching:
                self.searching = True
                self.get_logger().info('사람을 놓침 -> 마지막 방향으로 주변 탐색 시작')
            # 전반부는 마지막으로 향했던 방향, 후반부는 반대 방향으로 훑어서
            # 더 넓은 범위를 탐색한다 (한쪽으로만 계속 돌면 반대편은 못 봄)
            direction = (
                self.last_turn_direction
                if search_elapsed < self.search_timeout / 2.0
                else -self.last_turn_direction
            )
            scan_twist = Twist()
            scan_twist.angular.z = self.search_angular_speed * direction
            self.pub.publish(scan_twist)
        else:
            if self.searching:
                self.searching = False
                self.get_logger().info('탐색 시간 초과 -> 정지')
            self.last_twist = Twist()
            self.pub.publish(self.last_twist)  # 전부 0 -> 정지


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
