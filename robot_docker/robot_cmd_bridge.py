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
    micro-ROS Agent / MCU command subscriber

실행:
    systemctl start robot-control-bridge
    종료: Ctrl+C
"""

import ast
import base64
import socket
import json
import math
import os
import threading
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
from std_srvs.srv import SetBool

HOST = "0.0.0.0"
PORT = 9999
COMMAND_TIMEOUT = 0.5
MAP_DIRECTORY = "/opt/robot-control/maps"
MAP_NAME = "orchard_map"
LAST_POSE_PATH = f"{MAP_DIRECTORY}/last_pose.json"

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
LIDAR_STALE_SEC = 2.0

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
        self.emergency_pub = self.create_publisher(
            Bool, "/cmd_bridge/emergency_stop", 10
        )

        self.lock = threading.Lock()

        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.last_cmd_time = 0.0
        # Publish one safety stop when a command lease expires, then release
        # /cmd_vel so another controller (for example Nav2) can own it.
        self._timeout_stop_published = False
        # Mapping/Nav2 and GUI/Follow are mutually exclusive motor owners.
        # While this lock is active, socket commands cannot overwrite Nav2's
        # /cmd_vel output. Emergency stop remains available in every mode.
        self.navigation_mode = False
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._remote_nav_state = "idle"
        self._remote_nav_active = False
        self._remote_nav_message = "대기 중"
        self._remote_nav_goal = None
        self._remote_nav_distance = None
        self._remote_nav_goal_handle = None
        self._remote_nav_cancel_requested = False
        self._remote_nav_owns_mode = False
        self._map_pose = None
        self._last_pose_saved_at = 0.0
        self._last_pose_save_error_at = 0.0

        self.front_blocked = False
        self.front_min_dist = 10.0
        self._last_scan_at = 0.0

        self.left_history = deque(maxlen=SMOOTHING_WINDOW)
        self.right_history = deque(maxlen=SMOOTHING_WINDOW)

        # 상태: 'NORMAL' / 'BACKING_UP' / 'AVOIDING' / 'ESCAPE_TURN'
        self.avoid_state = "NORMAL"
        self.avoid_state_start = time.time()
        self.avoid_cycle_start = time.time()  # NORMAL을 벗어난 시점 (갇힘 감지용)

        self._last_servo_pan_sent = None
        self._tilt_set = False
        self._last_runtime_health = None

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 10
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.create_timer(1.0, self._set_initial_tilt_once)
        self.create_timer(2.0, self._report_runtime_health)
        self.create_service(
            SetBool, "/cmd_bridge/navigation_mode", self._set_navigation_mode
        )

        self.get_logger().info("cmd_bridge_node 시작됨 (갇힘 감지 포함)")

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped):
        pose = message.pose.pose
        quaternion = pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        saved_pose = {
            "x": round(float(pose.position.x), 4),
            "y": round(float(pose.position.y), 4),
            "yaw": round(float(yaw), 4),
        }
        now = time.monotonic()
        with self.lock:
            self._map_pose = saved_pose
            should_save = now - self._last_pose_saved_at >= 1.0
        if should_save:
            self._persist_map_pose(saved_pose, now)

    def _persist_map_pose(self, pose: dict, now: float):
        temporary_path = f"{LAST_POSE_PATH}.tmp"
        try:
            with open(temporary_path, "w", encoding="utf-8") as pose_file:
                json.dump(pose, pose_file, separators=(",", ":"))
                pose_file.write("\n")
            os.replace(temporary_path, LAST_POSE_PATH)
            with self.lock:
                self._last_pose_saved_at = now
        except OSError as error:
            if now - self._last_pose_save_error_at >= 30.0:
                self.get_logger().warn(f"마지막 지도 위치 저장 실패: {error}")
                self._last_pose_save_error_at = now

    def _report_runtime_health(self):
        cmd_subscribers = self.count_subscribers("/cmd_vel")
        cmd_publishers = self.count_publishers("/cmd_vel")
        scan_publishers = self.count_publishers("/scan")
        scan_age = (
            time.monotonic() - self._last_scan_at
            if self._last_scan_at > 0.0
            else float("inf")
        )
        scan_fresh = scan_age <= LIDAR_STALE_SEC
        health = (cmd_subscribers, cmd_publishers, scan_publishers, scan_fresh)
        if health == self._last_runtime_health:
            return
        self._last_runtime_health = health

        if cmd_subscribers == 0:
            self.get_logger().error(
                "모터 제어 연결 없음: /cmd_vel subscriber=0. "
                "micro-ROS Agent와 MCU serial session을 확인하세요."
            )
        elif cmd_subscribers > 1:
            self.get_logger().warn(
                f"중복 모터 subscriber 감지: /cmd_vel subscriber={cmd_subscribers}"
            )
        else:
            self.get_logger().info("모터 제어 연결 정상: /cmd_vel subscriber=1")

        if cmd_publishers > 1:
            self.get_logger().warn(
                f"속도 명령 publisher 경합 감지: /cmd_vel publisher={cmd_publishers}. "
                "GUI/Follow와 Navigation을 동시에 실행하지 마세요."
            )

        if scan_publishers == 0:
            self.get_logger().warn(
                "LiDAR 연결 없음: /scan publisher=0. 장애물 회피 데이터가 없습니다."
            )
        elif not scan_fresh:
            age_text = "수신 이력 없음" if math.isinf(scan_age) else f"{scan_age:.1f}초 지연"
            self.get_logger().warn(
                f"LiDAR 노드는 있으나 /scan 데이터가 없습니다: {age_text}. "
                "ESP32 radar publish와 Yahboom bringup을 확인하세요."
            )
        else:
            self.get_logger().info(
                f"LiDAR 데이터 정상: /scan publisher={scan_publishers}"
            )

    def _set_initial_tilt_once(self):
        if self._tilt_set:
            return
        msg = Int32()
        msg.data = SERVO_TILT_DEFAULT
        self.pub_servo_tilt.publish(msg)
        self.get_logger().info(f"카메라 틸트 초기화: servo_s2={SERVO_TILT_DEFAULT}")
        self._tilt_set = True

    def scan_callback(self, msg: LaserScan):
        self._last_scan_at = time.monotonic()
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
            if self.navigation_mode:
                return
            self.desired_linear = linear
            self.desired_angular = angular
            self.last_cmd_time = time.time()
            self._timeout_stop_published = False

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

    def _publish_timeout_stop_once(self, twist):
        """Stop once for an expired lease without racing a new command."""

        with self.lock:
            if time.time() - self.last_cmd_time <= COMMAND_TIMEOUT:
                return
            if self._timeout_stop_published:
                return
            self.pub.publish(twist)
            self._timeout_stop_published = True

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
        with self.lock:
            navigation_mode = self.navigation_mode
        if navigation_mode:
            # Nav2 owns /cmd_vel until the mapping/navigation controller
            # explicitly releases the lock.
            return
        notebook_connected = self._is_notebook_connected()

        if not notebook_connected:
            if self.avoid_state != "NORMAL":
                self._change_avoid_state("NORMAL")
            # A stale GUI lease must stop the robot, but continuously publishing
            # zero would overwrite Nav2's /cmd_vel commands forever. Send the
            # safety stop exactly once and then remain silent until a new GUI
            # command arrives.
            self._publish_timeout_stop_once(twist)
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

    def _set_navigation_mode(self, request, response):
        enabled = bool(request.data)
        self._set_navigation_control(enabled)
        response.success = True
        response.message = (
            "navigation owns motor control"
            if enabled
            else "GUI/follow motor control restored"
        )
        self.get_logger().info(response.message)
        return response

    def _set_navigation_control(self, enabled: bool):
        with self.lock:
            self.navigation_mode = enabled
            self.desired_linear = 0.0
            self.desired_angular = 0.0
            self.last_cmd_time = 0.0
            self._timeout_stop_published = True
        self.avoid_state = "NORMAL"
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)

    def navigation_snapshot(self):
        with self.lock:
            return {
                "state": self._remote_nav_state,
                "active": self._remote_nav_active,
                "message": self._remote_nav_message,
                "goal": None
                if self._remote_nav_goal is None
                else dict(self._remote_nav_goal),
                "distance_remaining": self._remote_nav_distance,
                "pose": None if self._map_pose is None else dict(self._map_pose),
            }

    def start_navigation(self, x: float, y: float, yaw: float):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("navigation coordinates must be finite")
        with self.lock:
            if self._remote_nav_active:
                raise ValueError("navigation goal is already active")
            self._remote_nav_state = "sending"
            self._remote_nav_active = True
            self._remote_nav_message = "Nav2 목표 전송 중"
            self._remote_nav_goal = {
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "yaw": round(float(yaw), 4),
            }
            self._remote_nav_distance = None
            self._remote_nav_goal_handle = None
            self._remote_nav_cancel_requested = False
            self._remote_nav_owns_mode = True

        if not self.navigator.wait_for_server(timeout_sec=0.5):
            self._finish_remote_navigation("error", "Nav2 action server가 준비되지 않음")
            raise ValueError("Nav2 action server is not ready")

        self._set_navigation_control(True)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        future = self.navigator.send_goal_async(
            goal, feedback_callback=self._on_navigation_feedback
        )
        future.add_done_callback(self._on_navigation_goal_response)

    def _on_navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_remote_navigation("error", f"Nav2 목표 전송 실패: {error}")
            return
        if not goal_handle.accepted:
            self._finish_remote_navigation("failed", "Nav2가 목표를 거부함")
            return

        with self.lock:
            self._remote_nav_goal_handle = goal_handle
            cancel_requested = self._remote_nav_cancel_requested
            if not cancel_requested:
                self._remote_nav_state = "navigating"
                self._remote_nav_message = "목표로 주행 중"
        if cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_message):
        distance = float(feedback_message.feedback.distance_remaining)
        with self.lock:
            self._remote_nav_distance = (
                round(distance, 3) if math.isfinite(distance) else None
            )

    def _on_navigation_result(self, future):
        try:
            status = future.result().status
        except Exception as error:
            self._finish_remote_navigation("error", f"Nav2 결과 수신 실패: {error}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_remote_navigation("succeeded", "목표 도착 완료")
        elif status == GoalStatus.STATUS_CANCELED:
            self._finish_remote_navigation("canceled", "Navigation 취소됨")
        else:
            self._finish_remote_navigation("failed", f"Navigation 실패 (status={status})")

    def _finish_remote_navigation(self, state: str, message: str):
        with self.lock:
            owns_mode = self._remote_nav_owns_mode
            self._remote_nav_state = state
            self._remote_nav_active = False
            self._remote_nav_message = message
            self._remote_nav_goal_handle = None
            self._remote_nav_cancel_requested = False
            self._remote_nav_owns_mode = False
        if owns_mode:
            self._set_navigation_control(False)
        self.get_logger().info(message)

    def cancel_navigation(self, reason="사용자 요청"):
        with self.lock:
            if not self._remote_nav_active:
                return False
            self._remote_nav_cancel_requested = True
            self._remote_nav_state = "canceling"
            self._remote_nav_message = f"Navigation 취소 중 ({reason})"
            goal_handle = self._remote_nav_goal_handle
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        return True

    def load_map_payload(self):
        image_path = f"{MAP_DIRECTORY}/{MAP_NAME}.pgm"
        yaml_path = f"{MAP_DIRECTORY}/{MAP_NAME}.yaml"
        with open(image_path, "rb") as image_file:
            image_data = image_file.read()
        metadata = {}
        with open(yaml_path, "r", encoding="utf-8") as yaml_file:
            for raw_line in yaml_file:
                line = raw_line.split("#", 1)[0].strip()
                if not line or ":" not in line:
                    continue
                key, value = (part.strip() for part in line.split(":", 1))
                metadata[key] = value
        origin = ast.literal_eval(metadata.get("origin", "[0, 0, 0]"))
        header_tokens = []
        for raw_line in image_data.splitlines():
            line = raw_line.split(b"#", 1)[0].strip()
            if line:
                header_tokens.extend(line.split())
            if len(header_tokens) >= 4:
                break
        if len(header_tokens) < 4 or header_tokens[0] not in {b"P2", b"P5"}:
            raise ValueError("invalid PGM map")
        return {
            "image_base64": base64.b64encode(image_data).decode("ascii"),
            "width": int(header_tokens[1]),
            "height": int(header_tokens[2]),
            "resolution": float(metadata["resolution"]),
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            "origin_yaw": float(origin[2]),
            "negate": int(metadata.get("negate", "0")),
            "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
            "free_thresh": float(metadata.get("free_thresh", "0.25")),
        }

    def _fill_with_desired(self, twist: Twist):
        with self.lock:
            twist.linear.x = float(self.desired_linear)
            twist.angular.z = float(self.desired_angular)

    def emergency_stop(self):
        self.cancel_navigation("긴급 정지")
        with self.lock:
            self.desired_linear = 0.0
            self.desired_angular = 0.0
            self.last_cmd_time = 0.0
            self._timeout_stop_published = True
        self.avoid_state = "NORMAL"
        self.avoid_state_start = time.time()
        stop = Twist()
        for _ in range(5):
            self.pub.publish(stop)
            time.sleep(0.05)
        event = Bool()
        event.data = True
        self.emergency_pub.publish(event)


def start_socket_server(node: CmdBridgeNode):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"cmd_bridge ready. listening on {PORT}", flush=True)

    while True:
        conn, addr = server.accept()
        print(f"클라이언트 연결됨: {addr}", flush=True)
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
                        response = handle_socket_command(node, cmd)
                    except (json.JSONDecodeError, TypeError, ValueError, OSError) as error:
                        response = {"ok": False, "error": str(error)}
                    conn.sendall(
                        json.dumps(response, separators=(",", ":")).encode("utf-8")
                        + b"\n"
                    )
        except ConnectionResetError:
            pass
        finally:
            print(f"클라이언트 연결 종료: {addr}", flush=True)
            node.emergency_stop()
            conn.close()


def handle_socket_command(node: CmdBridgeNode, cmd: dict):
    response = {"ok": True}
    command_type = cmd.get("type")
    if command_type == "navigate":
        node.start_navigation(float(cmd["x"]), float(cmd["y"]), float(cmd.get("yaw", 0.0)))
    elif command_type == "navigation_cancel":
        node.cancel_navigation()
    elif command_type == "map_request":
        response["map"] = node.load_map_payload()
    elif cmd.get("heartbeat") and not cmd.get("emergency_stop"):
        # Connection liveness is not a motor command. In particular, it must
        # not take /cmd_vel ownership away from a standalone Nav2 session.
        pass
    else:
        node.publish_cmd(
            cmd.get("linear", 0.0),
            cmd.get("angular", 0.0),
            cmd.get("servo_pan", None),
            bool(cmd.get("emergency_stop", False)),
        )
    response["navigation"] = node.navigation_snapshot()
    return response


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
