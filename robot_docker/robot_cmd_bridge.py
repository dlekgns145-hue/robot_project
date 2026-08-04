#!/usr/bin/env python3
"""
robot_cmd_bridge.py - 로봇(Docker 컨테이너) 안에서 실행하는 파일
------------------------------------------------------------------
[2026-07-15 갇힘 방지 로직 추가]
  벽 모서리 근처에서 BACKING_UP <-> AVOIDING 상태를 무한 반복하며
  "왔다갔다"만 하고 실제로 빠져나오지 못하는 문제 발견.
  (원인: 회전 중 옆벽이 다시 정면 감지 범위에 들어와서 계속 후진 재진입)

  해결: NORMAL이 아닌 상태로 STUCK_TIMEOUT_SEC 이상 머물면 "갇힘"으로
  판단하고, LiDAR 판단을 무시한 채 ESCAPE_TURN_TIME_SEC 동안 무조건
  제자리 회전(ESCAPE_TURN)한 뒤 NORMAL로 복귀. 이후 YOLO(노트북)의
  탐색 로직이 다시 정상적으로 명령을 내릴 수 있게 됨.

구조:
    데스크톱 GUI -> Ubuntu VM gateway
        --- TCP 소켓 (포트 9999) --->
    로봇 Docker 컨테이너 (이 스크립트)
        --- rclpy로 /cmd_vel 발행 (0.1초 타이머, 단일 지점) --->
        --- rclpy로 /servo_s1(좌우 카메라 팬) 발행 --->
    base_node_X3 / YB_Car_Node

실행:
    systemctl start robot-control-bridge
    종료: Ctrl+C
"""

import socket
import json
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

HOST = "0.0.0.0"
PORT = 9999
COMMAND_TIMEOUT = 0.5

OBSTACLE_AVOIDANCE_ENABLED = True
OBSTACLE_STOP_DISTANCE_M = 0.40
SAFE_CLEAR_DISTANCE_M = 0.65
FRONT_ANGLE_RANGE_DEG = 30
SIDE_ANGLE_MIN_DEG = 30
SIDE_ANGLE_MAX_DEG = 70
AVOID_LINEAR_SPEED = 0.12
AVOID_ANGULAR_SPEED = 0.35

BACKUP_TRIGGER_DISTANCE_M = 0.30
BACKUP_TARGET_DISTANCE_M = 0.55
BACKUP_MAX_TIME_SEC = 3.0
BACKUP_SPEED = -0.20

LIDAR_MIN_VALID_RANGE = 0.02

SMOOTHING_WINDOW = 5

# ---- 갇힘 감지 + 강제 탈출 (BACKING_UP <-> AVOIDING 무한 반복 방지) ----
STUCK_TIMEOUT_SEC = 4.0  # 이 시간 넘게 NORMAL로 못 돌아오면 "갇혔다"고 판단
ESCAPE_ANGULAR_SPEED = 0.4  # 탈출 회전 속도
ESCAPE_TURN_TIME_SEC = 2.5  # 이 시간 동안 LiDAR 무시하고 무조건 회전

SERVO_TILT_DEFAULT = -60
SERVO_PAN_MIN = -60
SERVO_PAN_MAX = 60


class CmdBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_bridge_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_servo_pan = self.create_publisher(Int32, "/servo_s1", 10)
        self.pub_servo_tilt = self.create_publisher(Int32, "/servo_s2", 10)

        self.lock = threading.Lock()

        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.last_cmd_time = 0.0

        self.front_blocked = False
        self.front_min_dist = 10.0

        self.left_history = deque(maxlen=SMOOTHING_WINDOW)
        self.right_history = deque(maxlen=SMOOTHING_WINDOW)

        # 상태: 'NORMAL' / 'BACKING_UP' / 'AVOIDING' / 'ESCAPE_TURN'
        self.avoid_state = "NORMAL"
        self.avoid_state_start = time.time()
        self.avoid_cycle_start = time.time()  # NORMAL을 벗어난 시점 (갇힘 감지용)

        self._last_servo_pan_sent = None
        self._tilt_set = False

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self._set_initial_tilt_once)

        self.get_logger().info("cmd_bridge_node 시작됨 (갇힘 감지 포함)")

    def _set_initial_tilt_once(self):
        if self._tilt_set:
            return
        msg = Int32()
        msg.data = SERVO_TILT_DEFAULT
        self.pub_servo_tilt.publish(msg)
        self.get_logger().info(f"카메라 틸트 초기화: servo_s2={SERVO_TILT_DEFAULT}")
        self._tilt_set = True

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        front_range_rad = math.radians(FRONT_ANGLE_RANGE_DEG)
        side_min_rad = math.radians(SIDE_ANGLE_MIN_DEG)
        side_max_rad = math.radians(SIDE_ANGLE_MAX_DEG)

        front_min_dist = float("inf")
        left_dists = []
        right_dists = []

        for i, r in enumerate(msg.ranges):
            if r < LIDAR_MIN_VALID_RANGE or math.isinf(r) or math.isnan(r):
                continue
            angle = angle_min + i * angle_increment
            if -front_range_rad <= angle <= front_range_rad:
                if r < front_min_dist:
                    front_min_dist = r
            if side_min_rad <= angle <= side_max_rad:
                right_dists.append(r)
            if -side_max_rad <= angle <= -side_min_rad:
                left_dists.append(r)

        self.front_blocked = front_min_dist < OBSTACLE_STOP_DISTANCE_M
        self.front_min_dist = front_min_dist

        if left_dists:
            self.left_history.append(sum(left_dists) / len(left_dists))
        if right_dists:
            self.right_history.append(sum(right_dists) / len(right_dists))

    def _get_smoothed_left(self):
        return (
            sum(self.left_history) / len(self.left_history)
            if self.left_history
            else None
        )

    def _get_smoothed_right(self):
        return (
            sum(self.right_history) / len(self.right_history)
            if self.right_history
            else None
        )

    def _change_avoid_state(self, new_state):
        self.avoid_state = new_state
        self.avoid_state_start = time.time()
        self.get_logger().warn(
            f"회피 상태 변경: {new_state} (정면거리={self.front_min_dist:.3f}m)"
        )

    def publish_cmd(self, linear, angular, servo_pan=None, emergency_stop=False):
        if emergency_stop:
            self.emergency_stop()
            return
        with self.lock:
            self.desired_linear = linear
            self.desired_angular = angular
            self.last_cmd_time = time.time()

        if servo_pan is not None:
            pan = max(SERVO_PAN_MIN, min(SERVO_PAN_MAX, int(servo_pan)))
            if pan != self._last_servo_pan_sent:
                msg = Int32()
                msg.data = pan
                self.pub_servo_pan.publish(msg)
                self._last_servo_pan_sent = pan

    def _is_notebook_connected(self):
        with self.lock:
            elapsed = time.time() - self.last_cmd_time
        return elapsed <= COMMAND_TIMEOUT

    def _choose_avoid_direction(self, left, right):
        if left is not None and right is not None:
            return "LEFT" if left > right else "RIGHT"
        elif left is not None:
            return "LEFT"
        elif right is not None:
            return "RIGHT"
        else:
            return None

    def control_loop(self):
        twist = Twist()
        notebook_connected = self._is_notebook_connected()

        if not notebook_connected:
            if self.avoid_state != "NORMAL":
                self._change_avoid_state("NORMAL")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            return

        if not OBSTACLE_AVOIDANCE_ENABLED:
            self._fill_with_desired(twist)
            self.pub.publish(twist)
            return

        with self.lock:
            desired_linear = self.desired_linear
            desired_angular = self.desired_angular
        stopped_by_operator = abs(desired_linear) < 1e-6 and abs(desired_angular) < 1e-6
        if stopped_by_operator:
            if self.avoid_state != "NORMAL":
                self._change_avoid_state("NORMAL")
            self.pub.publish(twist)
            return

        if self.avoid_state == "NORMAL" and self.front_blocked and desired_linear > 0.0:
            self.avoid_cycle_start = time.time()
            if self.front_min_dist < BACKUP_TRIGGER_DISTANCE_M:
                self._change_avoid_state("BACKING_UP")
            else:
                self._change_avoid_state("AVOIDING")

        # ---- 갇힘 감지: NORMAL이 아닌 상태로 너무 오래 있으면 강제 탈출 ----
        if self.avoid_state not in ("NORMAL", "ESCAPE_TURN"):
            if time.time() - self.avoid_cycle_start > STUCK_TIMEOUT_SEC:
                self._change_avoid_state("ESCAPE_TURN")
                self.get_logger().warn("갇힘 감지 - 강제 탈출 회전 시작 (LiDAR 무시)")

        elapsed = time.time() - self.avoid_state_start

        if self.avoid_state == "NORMAL":
            self._fill_with_desired(twist)

        elif self.avoid_state == "ESCAPE_TURN":
            if elapsed < ESCAPE_TURN_TIME_SEC:
                twist.linear.x = 0.0
                twist.angular.z = ESCAPE_ANGULAR_SPEED
                self.get_logger().info(
                    f"강제 탈출 회전 중... ({elapsed:.1f}/{ESCAPE_TURN_TIME_SEC}s)"
                )
            else:
                self._change_avoid_state("NORMAL")
                self.get_logger().info(
                    "탈출 회전 완료 - 일반 모드 복귀 (YOLO가 재탐색)"
                )
                self._fill_with_desired(twist)

        elif self.avoid_state == "BACKING_UP":
            if (
                self.front_min_dist < BACKUP_TARGET_DISTANCE_M
                and elapsed < BACKUP_MAX_TIME_SEC
            ):
                twist.linear.x = BACKUP_SPEED
                twist.angular.z = 0.0
                self.get_logger().info(
                    f"후진 중... ({elapsed:.1f}s) 정면={self.front_min_dist:.3f}m "
                    f"-> 목표 {BACKUP_TARGET_DISTANCE_M}m"
                )
            else:
                self._change_avoid_state("AVOIDING")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        elif self.avoid_state == "AVOIDING":
            if self.front_min_dist >= SAFE_CLEAR_DISTANCE_M:
                self._change_avoid_state("NORMAL")
                self.get_logger().info("정면 확보됨 - 일반 추적 복귀")
                self._fill_with_desired(twist)
            elif self.front_min_dist < BACKUP_TRIGGER_DISTANCE_M:
                self._change_avoid_state("BACKING_UP")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                left = self._get_smoothed_left()
                right = self._get_smoothed_right()
                direction = self._choose_avoid_direction(left, right)

                turn_only = self.front_min_dist < (OBSTACLE_STOP_DISTANCE_M + 0.1)

                if direction == "LEFT":
                    twist.linear.x = 0.0 if turn_only else AVOID_LINEAR_SPEED
                    twist.angular.z = AVOID_ANGULAR_SPEED
                elif direction == "RIGHT":
                    twist.linear.x = 0.0 if turn_only else AVOID_LINEAR_SPEED
                    twist.angular.z = -AVOID_ANGULAR_SPEED
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                left_str = f"{left:.2f}m" if left is not None else "없음"
                right_str = f"{right:.2f}m" if right is not None else "없음"
                mode_str = "회전만" if turn_only else "회전+전진"
                self.get_logger().info(
                    f"우회 중({mode_str}) -> {direction or '판단불가(정지)'} "
                    f"(좌={left_str}, 우={right_str}, 정면={self.front_min_dist:.3f}m)"
                )

        self.pub.publish(twist)

    def _fill_with_desired(self, twist: Twist):
        with self.lock:
            twist.linear.x = float(self.desired_linear)
            twist.angular.z = float(self.desired_angular)

    def emergency_stop(self):
        with self.lock:
            self.desired_linear = 0.0
            self.desired_angular = 0.0
            self.last_cmd_time = 0.0
        self.avoid_state = "NORMAL"
        self.avoid_state_start = time.time()
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.05)


def start_socket_server(node: CmdBridgeNode):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"cmd_bridge ready. listening on {PORT}")

    while True:
        conn, addr = server.accept()
        print(f"클라이언트 연결됨: {addr}")
        buffer = ""
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        cmd = json.loads(line)
                        node.publish_cmd(
                            cmd.get("linear", 0.0),
                            cmd.get("angular", 0.0),
                            cmd.get("servo_pan", None),
                            bool(cmd.get("emergency_stop", False)),
                        )
                    except json.JSONDecodeError:
                        print(f"잘못된 명령 무시: {line}")
        except ConnectionResetError:
            pass
        finally:
            print(f"클라이언트 연결 종료: {addr}")
            node.emergency_stop()
            conn.close()


def main():
    rclpy.init()
    node = CmdBridgeNode()

    server_thread = threading.Thread(
        target=start_socket_server, args=(node,), daemon=True
    )
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n종료 신호 감지 - 정지 명령 전송 중...")
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("정상 종료되었습니다.")


if __name__ == "__main__":
    main()
