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
import hashlib
import socket
import json
import math
import os
import threading
import time
import zlib
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger

from map_bundle import active_navigation_paths, install_navigation_bundle

HOST = "0.0.0.0"
PORT = 9999
COMMAND_TIMEOUT = 0.5
SERVER_COMMAND_TIMEOUT = float(os.getenv("SERVER_COMMAND_TIMEOUT_SEC", "0.5"))
SERVER_MAX_LINEAR_SPEED = float(os.getenv("SERVER_MAX_LINEAR_SPEED", "0.12"))
SERVER_MIN_LINEAR_SPEED = float(os.getenv("SERVER_MIN_LINEAR_SPEED", "0.04"))
SERVER_MAX_ANGULAR_SPEED = float(os.getenv("SERVER_MAX_ANGULAR_SPEED", "0.18"))
SERVER_MIN_ANGULAR_SPEED = float(os.getenv("SERVER_MIN_ANGULAR_SPEED", "0.18"))
SERVER_PURE_ROTATION_LINEAR_EPSILON = float(
    os.getenv("SERVER_PURE_ROTATION_LINEAR_EPSILON", "0.01")
)
SERVER_DRIVING_ANGULAR_DEADBAND = float(
    os.getenv("SERVER_DRIVING_ANGULAR_DEADBAND", "0.03")
)
NAVIGATION_LEASE_TIMEOUT = float(os.getenv("NAVIGATION_LEASE_TIMEOUT_SEC", "2.5"))
# The goal-accept response for /navigate_to_pose is relayed through the Zenoh
# bridge to the server's bt_navigator and back. That extra hop has been
# observed to silently drop the accept response at the RTPS layer (a FastDDS
# reader-history size mismatch), which left navigation stuck at "Nav2 목표
# 전송 중" forever with no error and no way to retry from the GUI. Time the
# "sending" state out instead of waiting on a response that may never come.
REMOTE_NAV_SEND_TIMEOUT = float(os.getenv("REMOTE_NAV_SEND_TIMEOUT_SEC", "8.0"))
MAP_DIRECTORY = "/opt/robot-control/maps"
MAP_NAME = "orchard_map"
LAST_POSE_PATH = f"{MAP_DIRECTORY}/last_pose.json"
HOME_POSE_PATH = os.getenv("HOME_POSE_PATH", f"{MAP_DIRECTORY}/home_pose.json")

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

# Follow Me 정지거리는 LiDAR(정확하지만 "이게 내가 따라가는 사람인지" 구분을
# 못 함)와 카메라(정확한 사람인지는 알지만 박스 비율은 대략치)를 같이 써서
# 정한다. LiDAR 정면거리만으로 목표 정지거리(약 70cm)를 판단했더니 가구가
# 많은 공간에서 사람이 아니라 옆 책상/의자 다리를 "이미 도착함"으로 오인해서
# 사람이 멀리 있어도 전진을 거부하는 문제가 있었다 (실기 확인, 2026-08-20).
# 그래서 두 단계로 나눈다:
#   1. FOLLOW_HARD_STOP_DISTANCE_M: 카메라 확인 없이 무조건 적용되는 근접
#      충돌 방지용 안전장치.
#   2. FOLLOW_TARGET_STOP_DISTANCE_M: "목표 지점 도착"으로 보고 정지하는
#      실제 목표 거리. LiDAR가 이 안쪽이라고 재도, follow_person.py가 보내는
#      카메라 박스 비율(FOLLOW_CAMERA_CONFIRM_RATIO 이상)이 같이 사람이
#      가깝다고 확인해줄 때만 실제로 멈춘다 -- 그 전까지는 LiDAR가 뭘
#      감지했든 그냥 가구려니 하고 계속 전진한다.
# 로봇 실측 폭(바퀴 포함)이 약 20cm로 작아서, follow 전용 안전거리는 일반
# 장애물 회피(OBSTACLE_STOP_DISTANCE_M=0.40m 등, 수동운전/매핑/내비게이션
# 공용)보다 더 타이트하게 잡아도 된다고 판단해 따로 낮췄다 (2026-08-20,
# 사용자 요청 -- follow 관련 거리만 줄이고 일반 장애물 회피는 그대로 둠).
FOLLOW_HARD_STOP_DISTANCE_M = 0.25
FOLLOW_TARGET_STOP_DISTANCE_M = 0.40
FOLLOW_CAMERA_CONFIRM_RATIO = 0.25

LIDAR_MIN_VALID_RANGE = 0.02
LIDAR_STALE_SEC = 2.0
REAR_HOUSING_MIN_ANGLE_DEG = -172.0
REAR_HOUSING_MAX_ANGLE_DEG = -164.0
REAR_HOUSING_MAX_RANGE_M = 0.16
CAMERA_STALE_SEC = 1.0
CAMERA_STOP_DISTANCE_M = 0.45
CAMERA_CRITICAL_DISTANCE_M = 0.30
# Above SERVER_MIN_LINEAR_SPEED's measured startup deadzone floor so a creep
# command actually overcomes drivetrain friction instead of stalling in
# place, but still far below normal driving speed.
CAMERA_CRITICAL_CREEP_SPEED = 0.05
# The full 70-degree camera scan belongs in Nav2's local costmap so DWB can
# steer around chair and desk legs.  The bridge is only the last-resort hard
# stop and must not collapse that spatial information into one broad
# "anything within 30 degrees stops all translation" decision.  Keep its
# independent stop corridor around the image centre; side obstacles remain in
# the costmap and are handled by trajectory collision checking.
CAMERA_HARD_STOP_HALF_ANGLE_DEG = 12.0

SMOOTHING_WINDOW = 5

# ---- 갇힘 감지 + 강제 탈출 (BACKING_UP <-> AVOIDING 무한 반복 방지) ----
STUCK_TIMEOUT_SEC = 4.0  # 이 시간 넘게 NORMAL로 못 돌아오면 "갇혔다"고 판단
ESCAPE_ANGULAR_SPEED = 0.4  # 탈출 회전 속도
ESCAPE_TURN_TIME_SEC = 2.5  # 이 시간 동안 LiDAR 무시하고 무조건 회전

# Keep the near floor and low steps visible while retaining enough forward
# view for normal driving in the classroom.
SERVO_TILT_DEFAULT = -55
SERVO_PAN_MIN = -60
SERVO_PAN_MAX = 60

# mapping-runtime을 컨테이너부터 새로 띄우는 경우, 이미지는 있으니 컨테이너
# 자체는 몇 초면 뜨지만 Nav2/SLAM 라이프사이클 노드들이 /autonomous_mapping/start
# 서비스를 등록하기까지는 실기 확인상 10~15초 정도 더 걸린다. 클라이언트 소켓
# 타임아웃(4초)보다 훨씬 기니까 이 대기는 반드시 별도 스레드에서 해야 한다.
MAPPING_RUNTIME_BOOT_TIMEOUT_SEC = 45.0
MAPPING_RUNTIME_COMPOSE_TIMEOUT_SEC = 60.0
# navigation-runtime도 mapping-runtime과 같은 이유로 컨테이너 기동 후 Nav2
# 라이프사이클 노드들이 /navigate_to_pose 액션 서버를 등록하기까지 시간이
# 걸린다 -- 지도 크기에 따라 costmap 초기화가 더 걸릴 수 있어 여유를 둔다.
NAVIGATION_RUNTIME_BOOT_TIMEOUT_SEC = 45.0


def is_rear_housing_reflection(angle_radians: float, distance: float) -> bool:
    """Ignore only the measured three-beam reflection from the robot body."""

    angle_degrees = math.degrees(angle_radians)
    return (
        REAR_HOUSING_MIN_ANGLE_DEG
        <= angle_degrees
        <= REAR_HOUSING_MAX_ANGLE_DEG
        and math.isfinite(distance)
        and distance <= REAR_HOUSING_MAX_RANGE_M
    )


def shape_server_velocity(linear: float, angular: float) -> tuple[float, float]:
    """Clamp Nav2 velocity without turning smooth curves into hard spins.

    The base needs roughly ``SERVER_MIN_ANGULAR_SPEED`` to start an in-place
    rotation.  Applying that minimum while the robot is also moving, however,
    converts every small DWB steering correction into the maximum turn rate.
    Keep the startup boost for pure rotation only and remove tiny steering
    noise while driving.
    """

    linear = max(-SERVER_MAX_LINEAR_SPEED, min(SERVER_MAX_LINEAR_SPEED, linear))
    angular = max(
        -SERVER_MAX_ANGULAR_SPEED, min(SERVER_MAX_ANGULAR_SPEED, angular)
    )
    # The physical base does not overcome drivetrain friction at the 0.013 m/s
    # commands produced by the smoother near a frontier. Preserve true zeros
    # and pure-rotation requests, but lift an intentional translation above
    # the measured startup deadzone. The local LiDAR/camera gate below still
    # blocks this command whenever the direction is unsafe.
    if (
        abs(linear) > SERVER_PURE_ROTATION_LINEAR_EPSILON
        and abs(linear) < SERVER_MIN_LINEAR_SPEED
    ):
        linear = math.copysign(SERVER_MIN_LINEAR_SPEED, linear)
    if abs(linear) <= SERVER_PURE_ROTATION_LINEAR_EPSILON:
        if 0.0 < abs(angular) < SERVER_MIN_ANGULAR_SPEED:
            angular = math.copysign(SERVER_MIN_ANGULAR_SPEED, angular)
    elif abs(angular) < SERVER_DRIVING_ANGULAR_DEADBAND:
        angular = 0.0
    return linear, angular


def validated_home_pose(payload: dict, expected_map_sha256: str) -> dict[str, float | str]:
    """Validate a home pose and bind it to exactly one occupancy map."""

    if not isinstance(payload, dict):
        raise ValueError("home pose must be a JSON object")
    pose_map_sha256 = str(payload.get("map_sha256") or "").lower()
    if len(pose_map_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in pose_map_sha256
    ):
        raise ValueError("home pose has no valid map_sha256")
    if pose_map_sha256 != expected_map_sha256.lower():
        raise ValueError("home pose belongs to a different map")
    try:
        x = float(payload["x"])
        y = float(payload["y"])
        yaw = float(payload["yaw"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("home pose requires numeric x, y, and yaw") from error
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("home pose coordinates must be finite")
    if abs(x) > 100.0 or abs(y) > 100.0:
        raise ValueError("home pose is outside the supported map range")
    return {
        "x": x,
        "y": y,
        "yaw": math.atan2(math.sin(yaw), math.cos(yaw)),
        "map_sha256": pose_map_sha256,
    }


class CmdBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_bridge_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_servo_pan = self.create_publisher(Int32, "/servo_s1", 10)
        self.pub_servo_tilt = self.create_publisher(Int32, "/servo_s2", 10)
        self.emergency_pub = self.create_publisher(
            Bool, "/cmd_bridge/emergency_stop", 10
        )
        self.safety_status_pub = self.create_publisher(
            String, "/cmd_bridge/safety_status", 10
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
        # Follow가 켜져 있는 동안, 앱은 화면 조작이 없어도 ~100ms마다
        # linear=0/angular=0인 실제 이동 명령을 계속 보낸다(heartbeat 플래그가
        # 없어서 일반 명령으로 처리됨). follow_cmd_callback은 ~200ms마다만
        # 갱신하므로, 이 유휴 폴링이 더 잦아서 follow의 명령을 거의 다
        # 덮어써버렸다 (실기 확인, 2026-08-20: /cmd_vel_follow는 계속 -0.4인데
        # 최종 /cmd_vel은 25개 중 1개만 -0.4). navigation_mode와 같은 패턴으로,
        # follow가 모터를 갖고 있는 동안은 일반 경로의 명령을 무시한다.
        self.follow_enabled = False
        self.server_linear = 0.0
        self.server_angular = 0.0
        self.last_server_cmd_at = 0.0
        self.last_navigation_lease_at = 0.0
        self.navigation_lease_active = False
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._remote_nav_state = "idle"
        self._remote_nav_active = False
        self._remote_nav_message = "대기 중"
        self._remote_nav_goal = None
        self._remote_nav_distance = None
        self._remote_nav_goal_handle = None
        self._remote_nav_cancel_requested = False
        self._remote_nav_owns_mode = False
        self._remote_nav_sending_since = 0.0
        self._map_pose = None
        self._mapping_status = {
            "state": "unavailable",
            "enabled": False,
            "message": "매핑 런타임을 기다리는 중",
            "saved_map": "",
        }
        self._mapping_autostart_in_progress = False
        self._navigation_autostart_in_progress = False
        self._last_pose_saved_at = 0.0
        self._last_pose_save_error_at = 0.0
        self._navigation_bundle = self._load_navigation_bundle_status()
        self._loadcell_status = {
            "connected": False,
            "grams": None,
            "state": "unavailable",
            "low_threshold_g": None,
            "high_threshold_g": None,
            "age_s": None,
        }
        self._apple_detection_status = {
            "connected": False,
            "model_ready": False,
            "source": "robot_camera",
            "state": "unavailable",
            "healthy_count": 0,
            "damaged_count": 0,
            "total_count": 0,
            "best_confidence": None,
            "inference_ms": None,
            "age_s": None,
            "last_error": None,
        }

        self.front_blocked = False
        self.front_min_dist = 10.0
        self.rear_min_dist = 10.0
        self._last_scan_at = 0.0
        self.camera_front_min_dist = float("inf")
        self.camera_nearest_dist = float("inf")
        self.camera_nearest_bearing_deg = None
        self._last_camera_scan_at = 0.0
        self._navigation_safety_reason = "inactive"
        self._requested_navigation_linear = 0.0
        self._requested_navigation_angular = 0.0
        self._output_navigation_linear = 0.0
        self._output_navigation_angular = 0.0

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
            LaserScan, "/camera_scan", self.camera_scan_callback, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_server", self.server_cmd_callback, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_follow", self.follow_cmd_callback, 10
        )
        # follow_person.py가 보내는 현재 타겟의 화면상 박스 높이 비율 --
        # LiDAR가 정면에서 가까운 걸 감지했을 때 그게 실제로 따라가는
        # 사람인지 확인하는 용도로만 쓴다 (follow_cmd_callback 참고).
        self.follow_camera_ratio = 0.0
        self.create_subscription(
            Float32, "/follow_person/box_height_ratio",
            self._follow_camera_ratio_callback, 10
        )
        self.create_subscription(
            Bool, "/cmd_bridge/navigation_lease", self.navigation_lease_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, 10
        )
        mapping_qos = QoSProfile(depth=1)
        mapping_qos.reliability = ReliabilityPolicy.RELIABLE
        mapping_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            "/autonomous_mapping/status",
            self._mapping_status_callback,
            mapping_qos,
        )
        self.create_subscription(
            String,
            "/loadcell_guard/status",
            self._loadcell_status_callback,
            mapping_qos,
        )
        self.create_subscription(
            String,
            "/apple_detection/status",
            self._apple_detection_status_callback,
            mapping_qos,
        )
        self.mapping_clients = {
            "mapping_start": self.create_client(
                Trigger, "/autonomous_mapping/start"
            ),
            "mapping_stop": self.create_client(
                Trigger, "/autonomous_mapping/stop"
            ),
            "mapping_save": self.create_client(
                Trigger, "/autonomous_mapping/save"
            ),
            "mapping_preview": self.create_client(
                Trigger, "/autonomous_mapping/preview"
            ),
        }
        self.follow_clients = {
            "follow_start": self.create_client(Trigger, "/follow_person/start"),
            "follow_stop": self.create_client(Trigger, "/follow_person/stop"),
        }
        self.follow_reset_target_client = self.create_client(
            Trigger, "/person_detection/reset_target"
        )
        self.person_detection_pause_client = self.create_client(
            Trigger, "/person_detection/pause"
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self._publish_navigation_safety_status)
        self.create_timer(1.0, self._set_initial_tilt_once)
        self.create_timer(2.0, self._report_runtime_health)
        self.create_service(
            SetBool, "/cmd_bridge/navigation_mode", self._set_navigation_mode
        )

        self.get_logger().info("cmd_bridge_node 시작됨 (갇힘 감지 포함)")

    def _mapping_status_callback(self, message: String):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self._mapping_status = payload

    def mapping_snapshot(self):
        with self.lock:
            return dict(self._mapping_status)

    def _loadcell_status_callback(self, message: String):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self._loadcell_status = payload

    def loadcell_snapshot(self):
        with self.lock:
            return dict(self._loadcell_status)

    def _apple_detection_status_callback(self, message: String):
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        # Bounding boxes belong on the MJPEG overlay and can make every TCP
        # heartbeat unnecessarily large. The app only needs the summary.
        payload.pop("boxes", None)
        with self.lock:
            self._apple_detection_status = payload

    def apple_detection_snapshot(self):
        with self.lock:
            return dict(self._apple_detection_status)

    @staticmethod
    def _load_navigation_bundle_status():
        path = f"{MAP_DIRECTORY}/navigation_bundle_status.json"
        try:
            with open(path, "r", encoding="utf-8") as status_file:
                payload = json.load(status_file)
            return payload if isinstance(payload, dict) else None
        except (OSError, ValueError, TypeError):
            return None

    def install_corrected_map(self, bundle):
        with self.lock:
            mapping_active = bool(self._mapping_status.get("enabled", False))
            navigation_active = self._remote_nav_active
        if mapping_active:
            raise RuntimeError("cannot install a corrected map while mapping is active")
        if navigation_active:
            raise RuntimeError("cannot install a corrected map while navigation is active")
        manifest = install_navigation_bundle(bundle, MAP_DIRECTORY)
        with self.lock:
            self._navigation_bundle = dict(manifest)
        return manifest

    @staticmethod
    def _call_runtime_launcher(action: str):
        """mapping_runtime_launcher.py(호스트에서 systemd로 도는 별도 프로세스)에게
        좁은 유닉스 소켓 하나로 docker compose 액션을 요청한다. mapping-runtime과
        navigation-runtime 둘 다 이 하나의 런처를 거친다. 반환값은
        {"ok": bool, "error": str 또는 None}.
        """
        socket_path = os.environ.get(
            "MAPPING_LAUNCHER_SOCKET", "/run/mapping_launcher.sock"
        )
        try:
            launcher_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            launcher_sock.settimeout(MAPPING_RUNTIME_COMPOSE_TIMEOUT_SEC)
            launcher_sock.connect(socket_path)
            launcher_sock.sendall(json.dumps({"action": action}).encode("utf-8"))
            launcher_sock.shutdown(socket.SHUT_WR)
            raw_response = b""
            while True:
                chunk = launcher_sock.recv(4096)
                if not chunk:
                    break
                raw_response += chunk
            launcher_sock.close()
            return json.loads(raw_response.decode("utf-8"))
        except (OSError, socket.timeout, json.JSONDecodeError) as error:
            return {
                "ok": False,
                "error": f"mapping_runtime_launcher 요청 실패 (런처가 안 떠 있을 수 있음): {error}",
            }

    def _stop_mapping_runtime_container(self):
        """mapping_stop이 성공한 뒤 mapping-runtime 컨테이너 자체를 내린다.

        /autonomous_mapping/stop은 탐색 동작만 멈출 뿐 Nav2/SLAM 스택이 로드된
        채로 컨테이너는 계속 떠 있어서, 그대로 잊고 두면 유휴 상태로도 CPU를
        50~70%씩 계속 잡아먹는다 (실기에서 두 번 발견, 2026-08-20) -- person-detect의
        CPU를 깎아먹어서 Follow Me 인식이 자꾸 끊기는 원인이 됐었다. mapping_start를
        다시 누르면 런처가 알아서 재기동하니 안전하게 내려도 된다. 소켓 핸들러
        스레드를 막지 않도록 항상 별도 데몬 스레드에서만 호출한다.
        """
        result = self._call_runtime_launcher("stop_mapping_runtime")
        if result.get("ok"):
            self.get_logger().info("mapping-runtime 컨테이너 정지 완료 (유휴 CPU 확보)")
        else:
            self.get_logger().error(
                f"mapping-runtime 컨테이너 정지 실패: {result.get('error')}"
            )

    def _launch_mapping_runtime_and_start(self):
        """mapping-runtime 컨테이너를 띄우고, 준비되는 대로 매핑을 시작한다.

        docker compose 실행과 라이프사이클 부팅 대기(최대
        MAPPING_RUNTIME_BOOT_TIMEOUT_SEC)가 합쳐지면 수십 초가 걸릴 수 있어
        소켓 핸들러 스레드를 막으면 안 된다. 이 메서드는 항상 별도의
        데몬 스레드에서만 호출한다 (mapping_command 참고).
        """
        try:
            launch_result = self._call_runtime_launcher("start_mapping_runtime")
            if not launch_result.get("ok"):
                self.get_logger().error(
                    f"mapping-runtime 컨테이너 시작 실패: {launch_result.get('error')}"
                )
                return
            self.get_logger().info("mapping-runtime 컨테이너 기동 완료, 서비스 준비 대기 중")

            client = self.mapping_clients["mapping_start"]
            if not client.wait_for_service(timeout_sec=MAPPING_RUNTIME_BOOT_TIMEOUT_SEC):
                self.get_logger().error(
                    "mapping-runtime이 시간 내에 /autonomous_mapping/start 서비스를 등록하지 않았습니다."
                )
                return

            future = client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(5.0):
                self.get_logger().error("mapping-runtime 자동 시작 요청이 응답하지 않았습니다.")
                return
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"mapping-runtime 자동 시작 완료: {response.message}")
            else:
                message = response.message if response is not None else "응답 없음"
                self.get_logger().error(f"mapping-runtime 자동 시작 실패: {message}")
        finally:
            with self.lock:
                self._mapping_autostart_in_progress = False

    def _stop_follow_for_mapping(self):
        """mapping_start가 들어오면 follow부터 내려서 CPU/모터 경합을 막는다.

        follow(YOLO+ReID+얼굴인식)와 mapping(Nav2/SLAM)은 둘 다 무겁고 둘 다
        /cmd_vel을 쓰려는 자율주행 소스라, 동시에 켜두면 CPU 경합은 물론 모터
        명령까지 서로 부딪힌다 (실기에서 "/cmd_vel publisher=2" 경고로 확인,
        2026-08-20). 앱(RobotViewModel)에는 이미 이 상호배타 로직이 있지만,
        앱을 거치지 않는 경로에서도 항상 지켜지도록 서버에서도 강제한다.
        follow가 애초에 안 돌고 있었으면 그냥 조용히 무시한다.
        """
        try:
            self.follow_command("follow_stop")
        except Exception:
            pass

    def mapping_command(self, command_type: str, timeout: float = 3.0):
        client = self.mapping_clients.get(command_type)
        if client is None:
            raise ValueError(f"unsupported mapping command: {command_type}")

        if command_type == "mapping_start":
            threading.Thread(
                target=self._stop_follow_for_mapping, daemon=True
            ).start()

        if not client.wait_for_service(timeout_sec=0.4):
            if command_type != "mapping_start":
                raise RuntimeError(
                    "매핑 런타임이 실행되지 않았습니다. mapping-runtime을 먼저 시작하세요."
                )
            with self.lock:
                already_in_progress = self._mapping_autostart_in_progress
                if not already_in_progress:
                    self._mapping_autostart_in_progress = True
            if not already_in_progress:
                threading.Thread(
                    target=self._launch_mapping_runtime_and_start, daemon=True
                ).start()
            return "매핑 런타임을 시작하는 중입니다. 준비되면 자동으로 매핑이 시작됩니다."

        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout):
            raise TimeoutError("mapping service response timeout")
        try:
            response = future.result()
        except Exception as error:
            raise RuntimeError(f"mapping service failed: {error}") from error
        if response is None:
            raise RuntimeError("mapping service returned no response")
        if not response.success:
            raise RuntimeError(str(response.message or "mapping command rejected"))
        if command_type == "mapping_stop":
            threading.Thread(
                target=self._stop_mapping_runtime_container, daemon=True
            ).start()
        return str(response.message or command_type)

    @staticmethod
    def _call_trigger(client, not_ready_message: str, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=0.4):
            raise RuntimeError(not_ready_message)
        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout):
            raise TimeoutError("service response timeout")
        try:
            response = future.result()
        except Exception as error:
            raise RuntimeError(f"service call failed: {error}") from error
        if response is None:
            raise RuntimeError("service returned no response")
        if not response.success:
            raise RuntimeError(str(response.message or "command rejected"))
        return str(response.message or "")

    def follow_command(self, command_type: str, timeout: float = 3.0):
        client = self.follow_clients.get(command_type)
        if client is None:
            raise ValueError(f"unsupported follow command: {command_type}")
        if command_type == "follow_start":
            # "따라와" = 말한 사람이 카메라에서 제일 가까운 사람이라고 가정하고,
            # 매번 새로 시작할 때마다 이전에 따라가던 대상 정보를 지운다.
            self._call_trigger(
                self.follow_reset_target_client,
                "person-detect가 실행되지 않았습니다. person-detect를 먼저 시작하세요.",
                timeout,
            )
            # mapping-runtime(Nav2/SLAM 스택)은 매핑을 안 쓰고 있어도(enabled=False)
            # 컨테이너가 떠 있기만 하면 유휴 상태로도 CPU를 50~70%씩 계속 잡아먹어서
            # person-detect의 인식 속도를 깎아먹는다 (실기에서 두 번 확인,
            # 2026-08-20). mapping-runtime이 왜/언제 떠 있게 됐는지와 무관하게,
            # follow를 시작할 땐 무조건 내려서 CPU를 확보한다 -- 이미 꺼져있으면
            # docker compose stop은 그냥 아무 일도 안 하니 안전하다.
            threading.Thread(
                target=self._stop_mapping_runtime_container, daemon=True
            ).start()
        if command_type == "follow_stop":
            # 정지 요청이 들어오면 트리거 성공 여부와 무관하게 즉시 일반 명령
            # 경로(앱 폴링/GUI)에 모터를 돌려준다 -- 실패하거나 타임아웃 나도
            # follow_enabled=True로 눌러앉아 있으면 안 된다.
            with self.lock:
                self.follow_enabled = False
        message = self._call_trigger(
            client,
            "follow-runtime이 실행되지 않았습니다. follow-runtime을 먼저 시작하세요.",
            timeout,
        )
        if command_type == "follow_start":
            with self.lock:
                self.follow_enabled = True
        if command_type == "follow_stop":
            if self.person_detection_pause_client.wait_for_service(timeout_sec=0.2):
                try:
                    self._call_trigger(
                        self.person_detection_pause_client,
                        "person detection pause service unavailable",
                        timeout,
                    )
                except Exception as error:
                    self.get_logger().warn(
                        f"person detection could not be paused: {error}"
                    )
        return message or command_type

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
        rear_min_dist = float("inf")
        left_dists = []
        right_dists = []

        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_increment
            if (
                r < LIDAR_MIN_VALID_RANGE
                or math.isinf(r)
                or math.isnan(r)
                or is_rear_housing_reflection(angle, r)
            ):
                continue
            if -front_range_rad <= angle <= front_range_rad:
                if r < front_min_dist:
                    front_min_dist = r
            if abs(angle) >= math.radians(150.0):
                if r < rear_min_dist:
                    rear_min_dist = r
            if side_min_rad <= angle <= side_max_rad:
                right_dists.append(r)
            if -side_max_rad <= angle <= -side_min_rad:
                left_dists.append(r)

        self.front_blocked = front_min_dist < OBSTACLE_STOP_DISTANCE_M
        self.front_min_dist = front_min_dist
        self.rear_min_dist = rear_min_dist

        if left_dists:
            self.left_history.append(sum(left_dists) / len(left_dists))
        if right_dists:
            self.right_history.append(sum(right_dists) / len(right_dists))

    def camera_scan_callback(self, msg: LaserScan):
        """Track camera obstacles in the narrow emergency-stop corridor.

        The complete scan is consumed by Nav2.  Using the former +/-30 degree
        minimum here made a desk leg at the edge of the view stop every
        forward trajectory, including trajectories that curved away from it.
        """

        front_range_rad = math.radians(CAMERA_HARD_STOP_HALF_ANGLE_DEG)
        distances = []
        nearest_distance = float("inf")
        nearest_bearing = None
        for index, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            angle = msg.angle_min + index * msg.angle_increment
            if (
                math.isfinite(distance)
                and distance >= max(0.0, float(msg.range_min))
                and distance < nearest_distance
            ):
                nearest_distance = distance
                nearest_bearing = math.degrees(angle)
            if (
                -front_range_rad <= angle <= front_range_rad
                and math.isfinite(distance)
                and distance >= max(0.0, float(msg.range_min))
            ):
                distances.append(distance)
        self.camera_front_min_dist = min(distances, default=float("inf"))
        self.camera_nearest_dist = nearest_distance
        self.camera_nearest_bearing_deg = nearest_bearing
        self._last_camera_scan_at = time.monotonic()

    @staticmethod
    def _finite_or_none(value):
        value = float(value)
        return round(value, 4) if math.isfinite(value) else None

    def _record_navigation_safety(
        self,
        reason: str,
        requested_linear: float,
        requested_angular: float,
        output_linear: float,
        output_angular: float,
    ) -> None:
        self._navigation_safety_reason = reason
        self._requested_navigation_linear = requested_linear
        self._requested_navigation_angular = requested_angular
        self._output_navigation_linear = output_linear
        self._output_navigation_angular = output_angular

    def _publish_navigation_safety_status(self) -> None:
        with self.lock:
            navigation_mode = self.navigation_mode
        reason = self._navigation_safety_reason if navigation_mode else "inactive"
        message = String()
        message.data = json.dumps(
            {
                "navigation_mode": navigation_mode,
                "reason": reason,
                "requested_linear": round(self._requested_navigation_linear, 4),
                "requested_angular": round(self._requested_navigation_angular, 4),
                "output_linear": round(self._output_navigation_linear, 4),
                "output_angular": round(self._output_navigation_angular, 4),
                "lidar_front_m": self._finite_or_none(self.front_min_dist),
                "lidar_rear_m": self._finite_or_none(self.rear_min_dist),
                "camera_hard_stop_m": self._finite_or_none(
                    self.camera_front_min_dist
                ),
                "camera_nearest_m": self._finite_or_none(
                    self.camera_nearest_dist
                ),
                "camera_nearest_bearing_deg": (
                    None
                    if self.camera_nearest_bearing_deg is None
                    else round(float(self.camera_nearest_bearing_deg), 2)
                ),
            },
            separators=(",", ":"),
        )
        self.safety_status_pub.publish(message)

    def server_cmd_callback(self, msg: Twist):
        """Accept a short-lived velocity lease from the compute server."""

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().error("서버 속도 명령에 NaN/Inf가 있어 폐기함")
            return
        linear, angular = shape_server_velocity(linear, angular)
        now = time.monotonic()
        with self.lock:
            if not self.navigation_mode:
                return
            self.server_linear = linear
            self.server_angular = angular
            self.last_server_cmd_at = now
            # An explicit mapping lease authorizes this navigation session.
            # Once authorized, fresh Nav2 commands are also proof that the
            # owner is alive; refresh ownership independently of the mapper's
            # CPU-heavy frontier clustering callback.
            if self.navigation_lease_active:
                self.last_navigation_lease_at = now

    def follow_cmd_callback(self, msg: Twist):
        """Relay follow_person.py's velocity through the GUI drive path.

        follow_person.py used to publish straight to /cmd_vel, contending
        with this node's own publisher there. It also cannot use
        /cmd_vel_server -- that topic is gated to navigation_mode (Nav2/
        mapping), which is off while following, so the command would be
        silently dropped. Routing through publish_cmd() instead makes
        follow just another source of desired_linear/desired_angular, the
        same as a GUI joystick command -- it gets the same navigation_mode
        exclusivity, the same LiDAR avoid-state obstacle avoidance, and the
        same command-timeout safety stop if follow_person.py goes silent.
        """

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().error("follow 속도 명령에 NaN/Inf가 있어 폐기함")
            return
        # 정지 판단은 두 단계다 (angular는 항상 그대로 둬서, 멈춘 채로도
        # 사람 쪽을 계속 바라볼 수 있게 한다):
        #   1. 근접 안전정지 -- 카메라 확인 없이 무조건 적용. 뭐가 됐든 이
        #      거리 안쪽으로는 절대 안 들어간다.
        #   2. 목표 도착 정지 -- LiDAR가 가깝다고 재도, 카메라가 보내는 박스
        #      비율이 같이 사람이 가깝다고 확인해줄 때만 진짜로 멈춘다.
        #      그렇지 않으면(LiDAR만 가깝다고 하는 경우) 옆에 있는 가구를
        #      사람으로 오인한 것으로 보고 계속 전진한다.
        if linear > 0.0 and self.front_min_dist <= FOLLOW_HARD_STOP_DISTANCE_M:
            linear = 0.0
        elif (
            linear > 0.0
            and self.front_min_dist <= FOLLOW_TARGET_STOP_DISTANCE_M
            and self.follow_camera_ratio >= FOLLOW_CAMERA_CONFIRM_RATIO
        ):
            linear = 0.0
        self.publish_cmd(linear, angular, source="follow")

    def _follow_camera_ratio_callback(self, msg: Float32):
        self.follow_camera_ratio = float(msg.data)

    def navigation_lease_callback(self, message: Bool):
        """Own motors while the compute mapper renews a short-lived lease."""

        if not bool(message.data):
            with self.lock:
                lease_active = self.navigation_lease_active
            if lease_active:
                self._set_navigation_control(False)
            return
        now = time.monotonic()
        with self.lock:
            already_enabled = self.navigation_mode
            self.last_navigation_lease_at = now
            self.navigation_lease_active = True
        if not already_enabled:
            self._set_navigation_control(True, lease_active=True, lease_time=now)

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

    def publish_cmd(self, linear, angular, servo_pan=None, emergency_stop=False, source="generic"):
        if emergency_stop:
            self.emergency_stop()
            return
        with self.lock:
            if self.navigation_mode:
                return
            if self.follow_enabled and source != "follow":
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
        self._check_remote_nav_send_timeout()
        twist = Twist()
        with self.lock:
            navigation_mode = self.navigation_mode
            lease_active = self.navigation_lease_active
            lease_age = time.monotonic() - self.last_navigation_lease_at
        if navigation_mode and lease_active and lease_age > NAVIGATION_LEASE_TIMEOUT:
            self.get_logger().error("navigation lease expired; stopping robot")
            self._set_navigation_control(False)
            return
        if navigation_mode:
            self._server_navigation_control_loop(twist)
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

    def _server_navigation_control_loop(self, twist):
        """Apply the robot-local safety envelope to server Nav2 commands."""

        now = time.monotonic()
        with self.lock:
            command_age = now - self.last_server_cmd_at
            linear = self.server_linear
            angular = self.server_angular
        scan_age = now - self._last_scan_at
        if command_age > SERVER_COMMAND_TIMEOUT:
            self._record_navigation_safety(
                "command_stale", linear, angular, 0.0, 0.0
            )
            self.pub.publish(twist)
            return
        if scan_age > LIDAR_STALE_SEC:
            self._record_navigation_safety(
                "lidar_stale", linear, angular, 0.0, 0.0
            )
            self.pub.publish(twist)
            return

        camera_distance = float("inf")
        if now - self._last_camera_scan_at <= CAMERA_STALE_SEC:
            camera_distance = self.camera_front_min_dist
        # A circular footprint can safely turn in place without getting any
        # closer to a front obstacle, so pure rotation is exempt below.
        # Full stop for any translation at this range treated an uncertain
        # camera reading as an absolute wall regardless of direction, and
        # combined with Nav2's own spin collision check also refusing
        # rotation in the same tight quarters, the robot had no automatic
        # way out at all -- confirmed 2026-08-19 both by a real desk leg at
        # this exact range (LiDAR-corroborated) and by a camera-only false
        # positive at the same range with LiDAR clear, on the same stretch a
        # manual driver had already proven passable the day before with
        # slow, careful nudges. Creep through/away at a bounded low speed
        # instead of stopping dead; LiDAR's own front/rear gates below are
        # unaffected and still hard-stop on their own reading.
        camera_creeping = False
        if (
            camera_distance < CAMERA_CRITICAL_DISTANCE_M
            and abs(linear) > SERVER_PURE_ROTATION_LINEAR_EPSILON
        ):
            linear = math.copysign(
                min(abs(linear), CAMERA_CRITICAL_CREEP_SPEED), linear
            )
            camera_creeping = True

        reason = "camera_critical_creep" if camera_creeping else "clear"
        if linear > 0.0 and self.front_min_dist < OBSTACLE_STOP_DISTANCE_M:
            linear = 0.0
            reason = "lidar_front"
        elif (
            linear > 0.0
            and not camera_creeping
            and camera_distance < CAMERA_STOP_DISTANCE_M
        ):
            linear = 0.0
            reason = "camera_front"
        if linear < 0.0 and self.rear_min_dist < OBSTACLE_STOP_DISTANCE_M:
            linear = 0.0
            reason = "lidar_rear"
        twist.linear.x = linear
        twist.angular.z = angular
        self._record_navigation_safety(
            reason,
            self.server_linear,
            self.server_angular,
            twist.linear.x,
            twist.angular.z,
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

    def _set_navigation_control(
        self, enabled: bool, *, lease_active: bool = False, lease_time: float = 0.0
    ):
        with self.lock:
            self.navigation_mode = enabled
            self.navigation_lease_active = bool(enabled and lease_active)
            self.last_navigation_lease_at = lease_time if enabled else 0.0
            self.desired_linear = 0.0
            self.desired_angular = 0.0
            self.last_cmd_time = 0.0
            self._timeout_stop_published = True
            self.server_linear = 0.0
            self.server_angular = 0.0
            self.last_server_cmd_at = 0.0
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
                "map_bundle": None
                if self._navigation_bundle is None
                else dict(self._navigation_bundle),
            }

    def start_navigation(self, x: float, y: float, yaw: float):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("navigation coordinates must be finite")
        with self.lock:
            if self._remote_nav_active:
                old_handle = self._remote_nav_goal_handle
                if old_handle is not None:
                    try:
                        old_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._remote_nav_active = False
                self._remote_nav_goal_handle = None
            self._remote_nav_state = "sending"
            self._remote_nav_active = True
            self._remote_nav_message = "Nav2 목표 전송 중"
            self._remote_nav_sending_since = time.monotonic()
            self._remote_nav_goal = {
                "x": round(float(x), 4),
                "y": round(float(y), 4),
                "yaw": round(float(yaw), 4),
            }
            self._remote_nav_distance = None
            self._remote_nav_goal_handle = None
            self._remote_nav_cancel_requested = False
            self._remote_nav_owns_mode = True

        if not self.navigator.wait_for_server(timeout_sec=2.0):
            # navigation-runtime(Nav2 스택)은 mapping-runtime과 같은 이유로
            # 기본 비활성화돼 있어서(profiles: [navigation]) 아무도 켜준 적이
            # 없으면 액션 서버가 아예 없다. 예전엔 여기서 바로 에러를 던졌는데,
            # mapping-runtime엔 이미 만들어둔 자동 기동 로직을 여기도 똑같이
            # 적용해서 컨테이너부터 띄우고 준비되는 대로 이 목표를 자동으로
            # 보내게 한다. 클라이언트는 navigation_snapshot()으로 진행 상황을
            # 계속 폴링하니 여기서 그냥 에러로 끝낼 필요가 없다.
            with self.lock:
                already_in_progress = self._navigation_autostart_in_progress
                if not already_in_progress:
                    self._navigation_autostart_in_progress = True
                self._remote_nav_message = "navigation-runtime을 시작하는 중"
            if not already_in_progress:
                threading.Thread(
                    target=self._launch_navigation_runtime_and_navigate,
                    args=(x, y, yaw),
                    daemon=True,
                ).start()
            return

        self._send_navigation_goal(x, y, yaw)

    def _send_navigation_goal(self, x: float, y: float, yaw: float):
        try:
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
        except Exception as error:
            self._finish_remote_navigation("error", f"Nav2 목표 전송 예외: {error}")
            raise

    def _launch_navigation_runtime_and_navigate(self, x: float, y: float, yaw: float):
        """navigation-runtime 컨테이너를 띄우고, 준비되는 대로 목표를 전송한다.

        mapping-runtime의 _launch_mapping_runtime_and_start와 동일한 패턴 --
        컨테이너 기동+Nav2 라이프사이클 부팅이 합쳐지면 수십 초가 걸릴 수 있어
        소켓 핸들러 스레드를 막으면 안 된다. 항상 별도 데몬 스레드에서만
        호출한다 (start_navigation 참고).
        """
        try:
            launch_result = self._call_runtime_launcher("start_navigation_runtime")
            if not launch_result.get("ok"):
                self._finish_remote_navigation(
                    "error",
                    f"navigation-runtime 컨테이너 시작 실패: {launch_result.get('error')}",
                )
                return
            self.get_logger().info(
                "navigation-runtime 컨테이너 기동 완료, 액션 서버 준비 대기 중"
            )
            if not self.navigator.wait_for_server(
                timeout_sec=NAVIGATION_RUNTIME_BOOT_TIMEOUT_SEC
            ):
                self._finish_remote_navigation(
                    "error",
                    "navigation-runtime이 시간 내에 /navigate_to_pose 액션 서버를 등록하지 않았습니다.",
                )
                return
            self._send_navigation_goal(x, y, yaw)
        finally:
            with self.lock:
                self._navigation_autostart_in_progress = False

    @staticmethod
    def _active_map_image_path() -> str:
        active_paths = active_navigation_paths(MAP_DIRECTORY)
        if active_paths is not None:
            return str(active_paths[0].with_name("map.pgm"))
        return f"{MAP_DIRECTORY}/{MAP_NAME}.pgm"

    def _active_map_sha256(self) -> str:
        image_path = self._active_map_image_path()
        try:
            with open(image_path, "rb") as image_file:
                return hashlib.sha256(image_file.read()).hexdigest()
        except OSError as error:
            raise ValueError(f"active navigation map is unavailable: {error}") from error

    def save_home_pose(self):
        """Persist the current localized AMCL pose as this map's home."""

        with self.lock:
            pose = None if self._map_pose is None else dict(self._map_pose)
        if pose is None:
            raise ValueError("AMCL pose is unavailable; localize the robot before setting home")
        map_sha256 = self._active_map_sha256()
        payload = validated_home_pose(
            {**pose, "map_sha256": map_sha256}, map_sha256
        )
        directory = os.path.dirname(HOME_POSE_PATH)
        os.makedirs(directory, exist_ok=True)
        temporary = f"{HOME_POSE_PATH}.tmp-{os.getpid()}"
        try:
            with open(temporary, "w", encoding="utf-8") as home_file:
                json.dump(payload, home_file, ensure_ascii=False, separators=(",", ":"))
                home_file.write("\n")
                home_file.flush()
                os.fsync(home_file.fileno())
            os.replace(temporary, HOME_POSE_PATH)
        finally:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
        return payload

    def navigate_home(self):
        with self.lock:
            mapping_enabled = bool(self._mapping_status.get("enabled", False))
        if mapping_enabled:
            raise ValueError("cannot navigate home while autonomous mapping is active")
        try:
            self.follow_command("follow_stop", timeout=0.75)
        except (RuntimeError, TimeoutError, ValueError) as error:
            # Navigation owns /cmd_vel after start_navigation(). A stale follow node must still
            # be reported, but a deployment without that optional runtime may navigate home.
            self.get_logger().warn(f"follow stop before navigate-home was unavailable: {error}")
        expected_digest = self._active_map_sha256()
        try:
            with open(HOME_POSE_PATH, "r", encoding="utf-8") as home_file:
                payload = json.load(home_file)
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"home pose is unavailable: {error}") from error
        pose = validated_home_pose(payload, expected_digest)
        self.start_navigation(pose["x"], pose["y"], pose["yaw"])
        return pose

    def soft_pause(self):
        """Stop every motion owner without assigning a new navigation target."""

        try:
            self.follow_command("follow_stop", timeout=0.75)
        except (RuntimeError, TimeoutError, ValueError) as error:
            # A missing follow runtime must never prevent the velocity/nav stop below.
            self.get_logger().warn(f"follow stop during soft pause was unavailable: {error}")
        self.emergency_stop()
        return "robot paused"

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

    def _check_remote_nav_send_timeout(self):
        with self.lock:
            if self._remote_nav_state != "sending":
                return
            elapsed = time.monotonic() - self._remote_nav_sending_since
            if elapsed <= REMOTE_NAV_SEND_TIMEOUT:
                return
        self.get_logger().error(
            f"Nav2 목표 응답이 {REMOTE_NAV_SEND_TIMEOUT:.0f}초 넘게 오지 않음 "
            "(게이트웨이/Zenoh 브릿지 확인 필요)"
        )
        self._finish_remote_navigation(
            "error",
            f"Nav2 목표 응답 시간 초과 ({REMOTE_NAV_SEND_TIMEOUT:.0f}초) - 다시 시도해 주세요",
        )

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

    def load_map_payload(self, variant="active"):
        active_paths = None
        if variant != "raw":
            try:
                active_paths = active_navigation_paths(MAP_DIRECTORY)
            except ValueError:
                active_paths = None
        if active_paths is None:
            image_path = f"{MAP_DIRECTORY}/{MAP_NAME}.pgm"
            yaml_path = f"{MAP_DIRECTORY}/{MAP_NAME}.yaml"
            pose_path = f"{MAP_DIRECTORY}/{MAP_NAME}_pose.json"
            occupancy_source = "lidar_slam_only"
        else:
            yaml_path = str(active_paths[0])
            pose_path = str(active_paths[1])
            image_path = str(active_paths[0].with_name("map.pgm"))
            occupancy_source = "lidar_slam_server_postprocessed"
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
        payload = {
            # Orchard maps grow quickly.  Deflate the PGM before base64 so the
            # gateway can transfer large mostly-uniform occupancy grids.
            "image_base64": base64.b64encode(zlib.compress(image_data, 6)).decode(
                "ascii"
            ),
            "image_encoding": "zlib+base64",
            "width": int(header_tokens[1]),
            "height": int(header_tokens[2]),
            "resolution": float(metadata["resolution"]),
            "origin_x": float(origin[0]),
            "origin_y": float(origin[1]),
            "origin_yaw": float(origin[2]),
            "negate": int(metadata.get("negate", "0")),
            "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
            "free_thresh": float(metadata.get("free_thresh", "0.25")),
            "occupancy_source": occupancy_source,
            "navigation_safe": True,
        }
        try:
            with open(pose_path, "r", encoding="utf-8") as pose_file:
                robot_pose = json.load(pose_file)
            if active_paths is None:
                expected_digest = str(robot_pose.get("map_sha256") or "")
                if hashlib.sha256(image_data).hexdigest() != expected_digest:
                    raise ValueError("saved mapping pose belongs to a different PGM")
            payload["robot_pose"] = {
                "x": float(robot_pose["x"]),
                "y": float(robot_pose["y"]),
                "yaw": float(robot_pose["yaw"]),
            }
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass
        return payload

    def _fill_with_desired(self, twist: Twist):
        with self.lock:
            twist.linear.x = float(self.desired_linear)
            twist.angular.z = float(self.desired_angular)

    def emergency_stop(self):
        self.cancel_navigation("긴급 정지")
        with self.lock:
            self.navigation_mode = False
            # follow_person.py의 내부 상태는 아직 살아있을 수 있지만(정식
            # follow_stop은 별도로 보내야 함), 브릿지 쪽 모터 소유권만큼은
            # 긴급정지 즉시 일반 명령 경로로 돌려줘야 한다 -- 안 풀어주면
            # 긴급정지를 눌러도 앱의 일반 명령이 follow_enabled에 계속 막혀서
            # 조작이 안 되는 안전 공백이 생긴다.
            self.follow_enabled = False
            self.desired_linear = 0.0
            self.desired_angular = 0.0
            self.server_linear = 0.0
            self.server_angular = 0.0
            self.last_server_cmd_at = 0.0
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
                data = conn.recv(65_536)
                if not data:
                    break
                buffer += data.decode("utf-8")
                if len(buffer) > 32 * 1024 * 1024:
                    raise ValueError("command buffer exceeded 32 MiB")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        cmd = json.loads(line)
                        response = handle_socket_command(node, cmd)
                    except (
                        json.JSONDecodeError,
                        TypeError,
                        ValueError,
                        RuntimeError,
                        OSError,
                    ) as error:
                        response = {"ok": False, "error": str(error)}
                    conn.sendall(
                        json.dumps(response, separators=(",", ":")).encode("utf-8")
                        + b"\n"
                    )
        except OSError:
            # 클라이언트가 응답을 받기 전에 연결을 끊는 경우 (ConnectionResetError,
            # BrokenPipeError 등) -- 이 한 클라이언트의 연결이 어떻게 끊기든 바깥의
            # accept 루프 자체는 절대 죽으면 안 된다. 예전엔 ConnectionResetError만
            # 잡아서, conn.sendall()이 BrokenPipeError(EPIPE)로 실패하는 경우엔 이
            # 예외가 그대로 새어나가 start_socket_server 스레드 전체가 죽었다 --
            # 그러면 프로세스는 살아있어도 포트 9999는 완전히 응답 불능이 되고,
            # 다음 클라이언트는 전부 "connection refused"를 받는다 (실기에서 발생,
            # 2026-08-20, 컨테이너 수동 재시작 전까지 지속됨).
            pass
        except Exception as error:  # noqa: BLE001 -- 이 accept 루프는 절대 죽으면 안 됨
            print(f"클라이언트 처리 중 예상치 못한 오류: {error}", flush=True)
        finally:
            print(f"클라이언트 연결 종료: {addr}", flush=True)
            node.emergency_stop()
            conn.close()


def handle_socket_command(node: CmdBridgeNode, cmd: dict):
    response = {"ok": True}
    command_type = cmd.get("type")
    if command_type == "navigate":
        node.start_navigation(float(cmd["x"]), float(cmd["y"]), float(cmd.get("yaw", 0.0)))
    elif command_type == "navigate_home":
        response["home"] = node.navigate_home()
    elif command_type == "home_set":
        response["home"] = node.save_home_pose()
    elif command_type == "soft_pause":
        response["command_result"] = {
            "type": command_type,
            "ok": True,
            "message": node.soft_pause(),
        }
    elif command_type == "navigation_cancel":
        node.cancel_navigation()
    elif command_type == "map_request":
        response["map"] = node.load_map_payload(str(cmd.get("map_variant", "active")))
    elif command_type == "map_install":
        manifest = node.install_corrected_map(cmd.get("bundle"))
        response["map_install"] = {
            "installed": True,
            "job_id": manifest["job_id"],
            "corrected_sha256": manifest["corrected_sha256"],
            "message": "corrected map and transformed pose installed for navigation",
        }
    elif command_type in {
        "mapping_start",
        "mapping_stop",
        "mapping_save",
        "mapping_preview",
    }:
        try:
            message = node.mapping_command(command_type)
            response["command_result"] = {
                "type": command_type,
                "ok": True,
                "message": message,
            }
        except (RuntimeError, TimeoutError, ValueError) as error:
            response["command_result"] = {
                "type": command_type,
                "ok": False,
                "message": str(error),
            }
    elif command_type in {"follow_start", "follow_stop"}:
        try:
            message = node.follow_command(command_type)
            response["command_result"] = {
                "type": command_type,
                "ok": True,
                "message": message,
            }
        except (RuntimeError, TimeoutError, ValueError) as error:
            response["command_result"] = {
                "type": command_type,
                "ok": False,
                "message": str(error),
            }
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
    mapping_snapshot = getattr(node, "mapping_snapshot", None)
    if callable(mapping_snapshot):
        response["mapping"] = mapping_snapshot()
    loadcell_snapshot = getattr(node, "loadcell_snapshot", None)
    if callable(loadcell_snapshot):
        response["loadcell"] = loadcell_snapshot()
    apple_detection_snapshot = getattr(node, "apple_detection_snapshot", None)
    if callable(apple_detection_snapshot):
        response["apple_detection"] = apple_detection_snapshot()
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
