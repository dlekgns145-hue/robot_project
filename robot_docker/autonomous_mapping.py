#!/usr/bin/env python3
"""autonomous_mapping.py - 경계선(Frontier) 탐색 기반 자율 매핑 및 지능형 지도 자동 저장 노드
-----------------------------------------------------------------------------------------
[매핑 기능 커스텀/뜯어고치기 안내]
이 노드는 실시간 /map (OccupancyGrid) 데이터를 분석하여 탐색 가능한 미지의 경계선(Frontier) 후보를 발굴하고,
Nav2 액션 서버(`/navigate_to_pose`)를 통해 로봇을 자율적으로 이동시켜 지도를 넓혀갑니다.

[핵심 작동 원리 & 상태 머신 (State Machine)]
1. 'starting': 매핑 런타임 시작 후 /map 수신 및 위치 추정(TF) 초기화 대기 (startup_delay)
2. 'exploring': 경계선 탐색 중
   - /map 데이터 수신 -> frontier_core의 frontier_candidates()로 미탐색-주행가능 경계선 계산
   - 로봇 위치에서 이동 가능한 경계선 중 적절한 거리/장애물 이격을 가진 목표(Goal) 선정
   - Nav2 /navigate_to_pose 목표 전송 및 /cmd_bridge/navigation_lease 모터 임대 갱신
   - 특정 경계선 이동에 실패하면 해당 위치를 블랙리스트(blacklist)에 등록하여 재진입 방지
3. 'saving': 매핑 완료 또는 사용자의 저장 요청 시 /map_saver/save_map 서비스 호출 후 mapping_core의 promote_saved_map()으로 원자적 저장
4. 'completed' / 'idle': 탐색 완료 또는 수동 정지 상태

[주요 조정 매개변수 (Parameters)]
- decision_period: 매핑 판단 루프 주기 (기본 1.0초)
- minimum_frontier_length: 유효한 경계선 최소 길이 (기본 0.35m)
- maximum_goal_distance: 한 번에 이동할 탐색 목표 최대 거리 (기본 7.0m)
- frontier_goal_standoff: 경계선과 목표 지점 사이의 안전 이격 거리 (기본 0.35m)
- minimum_save_known_area_m2: 지도 저장을 허용할 최소 관측 면적 (기본 1.0m²)
"""

from __future__ import annotations

import json
import hashlib
import math
import os
from dataclasses import asdict
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
from std_msgs.msg import Bool, String, UInt16
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_core import (
    GridSpec,
    frontier_candidates,
    reachable_free_cell_indices,
)
from mapping_core import (
    GoalProgress,
    MapQuality,
    analyze_occupancy_grid,
    cleanup_saved_map,
    promote_saved_map,
    quality_failures,
)


class AutonomousMappingNode(Node):
    """자율 탐색 매핑 노드 메인 클래스"""

    def __init__(self) -> None:
        super().__init__("autonomous_mapping")
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("decision_period", 1.0)
        self.declare_parameter("startup_delay", 8.0)
        # Allow Nav2's progress checker and recovery tree to finish a bounded
        # spin/backup cycle before the frontier goal is cancelled.
        self.declare_parameter("goal_timeout", 120.0)
        # Classroom chair and desk legs can leave a valid-looking frontier
        # path that makes no physical progress. Switch regions promptly instead
        # of waiting almost a minute at each blocked aisle.
        self.declare_parameter("goal_progress_timeout", 25.0)
        self.declare_parameter("goal_progress_minimum_delta", 0.10)
        self.declare_parameter("goal_cancel_timeout", 8.0)
        self.declare_parameter("goal_retry_delay", 1.0)
        self.declare_parameter("frontier_plan_timeout", 8.0)
        self.declare_parameter("maximum_consecutive_goal_failures", 5)
        self.declare_parameter("maximum_map_age", 8.0)
        self.declare_parameter("maximum_pose_age", 2.0)
        # A resumed, malformed map previously satisfied the old 60 s / 1 m
        # completion gate after only a few short stages. Full mapping must
        # demonstrate sustained exploration before it can auto-save as done.
        self.declare_parameter("minimum_runtime", 180.0)
        self.declare_parameter("minimum_completion_travel_distance", 5.0)
        self.declare_parameter("maximum_runtime", 1800.0)
        self.declare_parameter("completion_stable_maps", 5)
        self.declare_parameter("minimum_frontier_length", 0.35)
        self.declare_parameter("minimum_completion_frontier_length", 0.10)
        self.declare_parameter("minimum_completion_goal_distance", 0.20)
        self.declare_parameter("minimum_goal_distance", 0.45)
        self.declare_parameter("maximum_goal_distance", 7.0)
        # Short 0.35 m stages left only about 0.15 m outside Nav2's goal
        # tolerance. On the physical base that was too little commanded travel
        # to overcome startup friction reliably, so the same nearby corridor
        # failed five times without exposing meaningful new scan area.
        self.declare_parameter("maximum_goal_step_distance", 0.75)
        self.declare_parameter("frontier_goal_standoff", 0.35)
        self.declare_parameter("minimum_goal_obstacle_clearance", 0.28)
        self.declare_parameter("maximum_robot_free_seed_distance", 0.50)
        self.declare_parameter("maximum_exploration_radius", 25.0)
        self.declare_parameter("blacklist_radius", 0.7)
        self.declare_parameter("staged_goal_blacklist_radius", 0.35)
        # One Nav2 progress attempt can last 55 seconds. A 20 second blacklist
        # let the same blocked approach become eligible almost immediately;
        # retain it long enough to force exploration of another direction.
        self.declare_parameter("failed_goal_blacklist_seconds", 120.0)
        # Reaching a gray boundary means that region has already received an
        # exploration attempt. Keep it out of the destination set for the rest
        # of this run; paths to genuinely new frontiers may still cross it.
        self.declare_parameter("reached_goal_blacklist_seconds", 1800.0)
        self.declare_parameter("minimum_start_known_area_m2", 0.25)
        self.declare_parameter("minimum_save_known_area_m2", 1.0)
        self.declare_parameter("minimum_save_free_area_m2", 0.50)
        self.declare_parameter("map_output", "/opt/robot-control/maps/orchard_map")
        self.declare_parameter("minimum_battery_percent", 25)
        self.declare_parameter("maximum_battery_age", 10.0)
        self.declare_parameter("resume_pose_graph", True)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        map_topic = str(self.get_parameter("map_topic").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self._on_map, map_qos
        )
        self.map_refresh_subscription = self.create_subscription(
            OccupancyGrid,
            "/autonomous_mapping/map_refresh",
            self._on_map,
            10,
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/autonomous_mapping/status", status_qos
        )
        self.goal_publisher = self.create_publisher(
            PoseStamped, "/autonomous_mapping/goal", 10
        )
        self.stop_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.path_planner = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.map_saver = self.create_client(SaveMap, "/map_saver/save_map")
        # Some FastDDS runs deliver SLAM's map to Nav2 but not to this Python
        # subscriber. The service exposes the same current OccupancyGrid and
        # provides a deterministic fallback.
        self.dynamic_map = self.create_client(GetMap, "/slam_toolbox/dynamic_map")
        self.pose_graph_saver = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )
        self.pose_graph_loader = self.create_client(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map"
        )
        self.navigation_lease_publisher = self.create_publisher(
            Bool, "/cmd_bridge/navigation_lease", 10
        )
        self.emergency_subscription = self.create_subscription(
            Bool,
            "/cmd_bridge/emergency_stop",
            self._on_emergency_stop,
            10,
        )
        self.battery_subscription = self.create_subscription(
            UInt16, "/battery", self._on_battery, 10
        )
        self.battery_refresh_subscription = self.create_subscription(
            UInt16,
            "/autonomous_mapping/battery_refresh",
            self._on_battery,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, "/autonomous_mapping/start", self._start)
        self.create_service(Trigger, "/autonomous_mapping/stop", self._stop)
        self.create_service(Trigger, "/autonomous_mapping/save", self._save)
        self.create_service(Trigger, "/autonomous_mapping/preview", self._preview)
        self.timer = self.create_timer(
            float(self.get_parameter("decision_period").value), self._tick
        )
        self.status_timer = self.create_timer(1.0, self._publish_periodic_status)

        self.latest_map: OccupancyGrid | None = None
        self.latest_map_quality: MapQuality | None = None
        self.last_map_received_at = 0.0
        self.map_request_future = None
        self.latest_battery_percent: int | None = None
        self.last_battery_received_at = 0.0
        self.map_sequence = 0
        self.last_empty_map_sequence = -1
        self.empty_map_count = 0
        self.active_frontier_count = 0
        self.reachable_frontier_count = 0
        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.state = "starting" if self.enabled else "idle"
        self.started_at = self._seconds() if self.enabled else 0.0
        self.start_pose: tuple[float, float] | None = None
        self.goal_candidate = None
        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_cancel_pending = False
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.goal_generation = 0
        self.goal_sent_at = 0.0
        self.goal_progress: GoalProgress | None = None
        self.plan_candidate = None
        self.plan_context: tuple[float, float, int] | None = None
        self.plan_handle = None
        self.plan_request_pending = False
        self.plan_generation = 0
        self.plan_sent_at = 0.0
        self.next_goal_not_before = 0.0
        self.consecutive_goal_failures = 0
        self.distance_remaining = None
        self.blacklist: list[tuple[float, float, float, bool]] = []
        self.staged_blacklist: list[tuple[float, float, float]] = []
        self.travel_distance = 0.0
        self.last_travel_pose: tuple[float, float] | None = None
        self.save_pending = False
        self.save_requested_reason = ""
        self.saving_map_output = ""
        self.saving_staging_output = ""
        self.saving_robot_pose = None
        self.save_attempt_sequence = 0
        self.last_save_error = ""
        self.saved_map = ""
        self.saved_pose = None
        self.save_sequence = 0
        self.pose_graph_save_requested = False
        self.pose_graph_save_pending = False
        self.pose_graph_save_sequence = 0
        self.pose_graph_staging_base = ""
        self.pose_graph_saved = False
        self.pose_graph_last_error = ""
        self.pose_graph_next_retry_at = 0.0
        self.pose_graph_resume_future = None
        self.pose_graph_resumed = False
        self.pose_graph_resume_state = self._initial_pose_graph_resume_state()
        self.status_message = "autonomous mapping ready; waiting for start"
        self.map_refresh_timer = self.create_timer(1.0, self._refresh_map)
        self.pose_graph_timer = self.create_timer(0.5, self._maintain_pose_graph)
        self._publish_status(self.status_message)

    def _seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.last_map_received_at = self._seconds()
        self.map_sequence += 1
        try:
            self.latest_map_quality = analyze_occupancy_grid(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution=float(message.info.resolution),
            )
        except (TypeError, ValueError) as error:
            self.latest_map_quality = None
            self.get_logger().error(f"invalid occupancy grid: {error}")

    def _on_battery(self, message: UInt16) -> None:
        self.latest_battery_percent = max(0, min(100, int(message.data)))
        self.last_battery_received_at = self._seconds()

    def _pose_graph_base(self) -> Path:
        return Path(f"{self.get_parameter('map_output').value}_slam")

    def _pose_graph_files(self, base: Path | None = None) -> tuple[Path, Path]:
        graph_base = self._pose_graph_base() if base is None else base
        return Path(f"{graph_base}.posegraph"), Path(f"{graph_base}.data")

    def _initial_pose_graph_resume_state(self) -> str:
        if not bool(self.get_parameter("resume_pose_graph").value):
            return "disabled"
        graph_path, data_path = self._pose_graph_files()
        pose_path = Path(f"{self.get_parameter('map_output').value}_pose.json")
        try:
            if all(
                path.is_file() and path.stat().st_size > 0
                for path in (graph_path, data_path, pose_path)
            ):
                return "pending"
        except OSError as error:
            self.pose_graph_last_error = f"pose graph files could not be inspected: {error}"
            return "failed"
        return "not_available"

    def _maintain_pose_graph(self) -> None:
        if self.pose_graph_resume_state == "pending":
            self._request_pose_graph_resume()
        if (
            self.pose_graph_save_requested
            and not self.pose_graph_save_pending
            and self._seconds() >= self.pose_graph_next_retry_at
        ):
            self._request_pose_graph_save()

    def _request_pose_graph_resume(self) -> None:
        if self.pose_graph_resume_future is not None:
            return
        if not self.pose_graph_loader.service_is_ready():
            return
        pose_path = Path(f"{self.get_parameter('map_output').value}_pose.json")
        try:
            pose = json.loads(pose_path.read_text(encoding="utf-8"))
            request = DeserializePoseGraph.Request()
            request.filename = str(self._pose_graph_base())
            request.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
            request.initial_pose.x = float(pose["x"])
            request.initial_pose.y = float(pose["y"])
            request.initial_pose.theta = float(pose["yaw"])
        except (OSError, KeyError, TypeError, ValueError) as error:
            self.pose_graph_resume_state = "failed"
            self.pose_graph_last_error = f"pose graph resume metadata invalid: {error}"
            self._publish_status(self.pose_graph_last_error)
            return
        self.latest_map = None
        self.latest_map_quality = None
        self.last_map_received_at = 0.0
        self.pose_graph_resume_state = "loading"
        try:
            future = self.pose_graph_loader.call_async(request)
        except Exception as error:
            self.pose_graph_resume_state = "failed"
            self.pose_graph_last_error = f"pose graph resume request failed: {error}"
            self._publish_status(self.pose_graph_last_error)
            return
        self.pose_graph_resume_future = future
        future.add_done_callback(self._on_pose_graph_resumed)

    def _on_pose_graph_resumed(self, future) -> None:
        self.pose_graph_resume_future = None
        try:
            future.result()
        except Exception as error:
            self.pose_graph_resume_state = "failed"
            self.pose_graph_last_error = f"pose graph resume failed: {error}"
            self._publish_status(self.pose_graph_last_error)
            return
        self.pose_graph_resumed = True
        self.pose_graph_resume_state = "resumed"
        self.pose_graph_last_error = ""
        self._publish_status("saved SLAM pose graph resumed; waiting for start")

    def _request_pose_graph_save(self) -> None:
        if not self.pose_graph_saver.service_is_ready():
            self.pose_graph_last_error = "pose graph saver service is not ready"
            self.pose_graph_next_retry_at = self._seconds() + 2.0
            return
        self.pose_graph_save_sequence += 1
        stable_base = self._pose_graph_base()
        staging_base = stable_base.with_name(
            f".{stable_base.name}.pending-{self.pose_graph_save_sequence}-{os.getpid()}"
        )
        for path in self._pose_graph_files(staging_base):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        request = SerializePoseGraph.Request()
        request.filename = str(staging_base)
        self.pose_graph_save_pending = True
        self.pose_graph_staging_base = str(staging_base)
        try:
            future = self.pose_graph_saver.call_async(request)
        except Exception as error:
            self.pose_graph_save_pending = False
            self.pose_graph_last_error = f"pose graph save request failed: {error}"
            self.pose_graph_next_retry_at = self._seconds() + 2.0
            return
        future.add_done_callback(self._on_pose_graph_saved)

    def _on_pose_graph_saved(self, future) -> None:
        self.pose_graph_save_pending = False
        staging_base = Path(self.pose_graph_staging_base)
        staging_files = self._pose_graph_files(staging_base)
        try:
            response = future.result()
            if int(response.result) != int(SerializePoseGraph.Response.RESULT_SUCCESS):
                raise ValueError(f"slam_toolbox result={int(response.result)}")
            if not all(path.is_file() and path.stat().st_size > 0 for path in staging_files):
                raise ValueError("slam_toolbox did not create both pose graph files")
            stable_files = self._pose_graph_files()
            for source, destination in zip(staging_files, stable_files):
                os.replace(source, destination)
            manifest = {
                "schema_version": 1,
                "saved_unix": self._seconds(),
                "posegraph_sha256": hashlib.sha256(stable_files[0].read_bytes()).hexdigest(),
                "data_sha256": hashlib.sha256(stable_files[1].read_bytes()).hexdigest(),
            }
            manifest_path = Path(f"{self._pose_graph_base()}_manifest.json")
            temporary = manifest_path.with_name(f".{manifest_path.name}.tmp-{os.getpid()}")
            temporary.write_text(json.dumps(manifest, separators=(",", ":")) + "\n", encoding="utf-8")
            os.replace(temporary, manifest_path)
        except Exception as error:
            for path in staging_files:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            self.pose_graph_last_error = f"pose graph save failed: {error}"
            self.pose_graph_next_retry_at = self._seconds() + 2.0
            self._publish_status(self.pose_graph_last_error)
            return
        self.pose_graph_save_requested = False
        self.pose_graph_saved = True
        self.pose_graph_last_error = ""
        self._publish_status("map and resumable SLAM pose graph saved")

    def _refresh_map(self) -> None:
        """Fetch SLAM's current grid if the DDS map stream is absent or stale."""
        if self.last_map_received_at > 0.0 and self._map_age() < 2.0:
            return
        if self.map_request_future is not None:
            return
        if not self.dynamic_map.service_is_ready():
            return
        future = self.dynamic_map.call_async(GetMap.Request())
        self.map_request_future = future
        future.add_done_callback(self._on_dynamic_map)

    def _on_dynamic_map(self, future) -> None:
        self.map_request_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warn(f"dynamic map request failed: {error}")
            return
        message = response.map
        if int(message.info.width) <= 0 or int(message.info.height) <= 0:
            return
        self._on_map(message)

    def _robot_pose_full(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.2)
        )
        stamp = transform.header.stamp
        pose_stamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        maximum_age = float(self.get_parameter("maximum_pose_age").value)
        pose_age = self._seconds() - pose_stamp
        if maximum_age > 0.0 and (pose_stamp <= 0.0 or pose_age > maximum_age):
            raise TransformException(
                f"latest {self.map_frame}->{self.base_frame} pose is stale "
                f"({pose_age:.2f} s)"
            )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), float(yaw)

    def _robot_pose(self) -> tuple[float, float]:
        x, y, _yaw = self._robot_pose_full()
        return x, y

    def _map_age(self, now: float | None = None) -> float:
        if self.last_map_received_at <= 0.0:
            return float("inf")
        return (self._seconds() if now is None else now) - self.last_map_received_at

    def _mapping_readiness_errors(self) -> list[str]:
        errors: list[str] = []
        if self.save_pending:
            errors.append("a map save is still in progress")
        if self.pose_graph_resume_state in {"pending", "loading"}:
            errors.append("saved SLAM pose graph is still loading")
        elif self.pose_graph_resume_state == "failed":
            errors.append(self.pose_graph_last_error or "saved SLAM pose graph could not be loaded")
        maximum_battery_age = float(
            self.get_parameter("maximum_battery_age").value
        )
        battery_age = (
            float("inf")
            if self.last_battery_received_at <= 0.0
            else self._seconds() - self.last_battery_received_at
        )
        if self.latest_battery_percent is None or (
            maximum_battery_age > 0.0 and battery_age > maximum_battery_age
        ):
            errors.append("a fresh battery reading is unavailable")
        elif self.latest_battery_percent <= int(
            self.get_parameter("minimum_battery_percent").value
        ):
            errors.append(
                f"battery is too low ({self.latest_battery_percent}%)"
            )
        if self.latest_map is None or self.latest_map_quality is None:
            errors.append("a valid occupancy grid has not arrived")
        else:
            maximum_age = float(self.get_parameter("maximum_map_age").value)
            age = self._map_age()
            if maximum_age > 0.0 and age > maximum_age:
                errors.append(f"occupancy grid is stale ({age:.1f} s)")
            minimum_start_area = float(
                self.get_parameter("minimum_start_known_area_m2").value
            )
            if self.latest_map_quality.known_area_m2 < minimum_start_area:
                errors.append(
                    "SLAM has not observed enough area "
                    f"({self.latest_map_quality.known_area_m2:.2f} < "
                    f"{minimum_start_area:.2f} m^2)"
                )
            if self.latest_map_quality.free_cells <= 0:
                errors.append("occupancy grid contains no known free cell")
        if not self.navigator.server_is_ready():
            errors.append("Nav2 action server is not ready")
        if not self.path_planner.server_is_ready():
            errors.append("Nav2 path planner is not ready")
        if not self.map_saver.service_is_ready():
            errors.append("map saver service is not ready")
        return errors

    def _save_quality_errors(self) -> list[str]:
        if self.latest_map_quality is None:
            return ["a valid occupancy grid is unavailable"]
        return quality_failures(
            self.latest_map_quality,
            minimum_known_area_m2=float(
                self.get_parameter("minimum_save_known_area_m2").value
            ),
            minimum_free_area_m2=float(
                self.get_parameter("minimum_save_free_area_m2").value
            ),
        )

    def _start(self, _request: Trigger.Request, response: Trigger.Response):
        if self.enabled:
            response.success = False
            response.message = "autonomous mapping is already active"
            return response
        readiness_errors = self._mapping_readiness_errors()
        if readiness_errors:
            response.success = False
            response.message = "mapping is not ready: " + "; ".join(readiness_errors)
            self.state = "not_ready"
            self._publish_status(response.message)
            return response
        try:
            self.start_pose = self._robot_pose()
        except TransformException as error:
            response.success = False
            response.message = f"map transform is not ready: {error}"
            return response
        try:
            spec = self._map_spec(self.latest_map)
            reachable = reachable_free_cell_indices(
                self.latest_map.data,
                spec,
                robot_x=self.start_pose[0],
                robot_y=self.start_pose[1],
                maximum_seed_distance=float(
                    self.get_parameter("maximum_robot_free_seed_distance").value
                ),
            )
        except ValueError as error:
            reachable = set()
            self.get_logger().error(f"mapping preflight grid failure: {error}")
        if not reachable:
            response.success = False
            response.message = (
                "mapping is not ready: robot pose is not connected to nearby "
                "known free map space"
            )
            self.state = "not_ready"
            self._publish_status(response.message)
            return response
        self._set_navigation_mode(True)
        self.enabled = True
        self.state = "starting"
        self.started_at = self._seconds()
        self.empty_map_count = 0
        self.last_empty_map_sequence = -1
        self.active_frontier_count = 0
        self.reachable_frontier_count = 0
        self.blacklist.clear()
        self.staged_blacklist.clear()
        self.goal_cancel_pending = False
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.goal_progress = None
        self.plan_candidate = None
        self.plan_context = None
        self.plan_handle = None
        self.plan_request_pending = False
        self.plan_generation += 1
        self.next_goal_not_before = 0.0
        self.consecutive_goal_failures = 0
        self.travel_distance = 0.0
        self.last_travel_pose = self.start_pose
        self.saved_map = ""
        self.saved_pose = None
        response.success = True
        response.message = "autonomous mapping started"
        self._publish_status(response.message)
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response):
        was_active = (
            self.enabled
            or self.goal_handle is not None
            or self.plan_request_pending
        )
        self._stop_exploration("stopped by operator")
        response.success = was_active
        response.message = "autonomous mapping stopped"
        return response

    def _save(self, _request: Trigger.Request, response: Trigger.Response):
        map_output = str(self.get_parameter("map_output").value)
        if self._request_map_save("manual save requested"):
            response.success = True
            response.message = f"saving map to {map_output}"
        else:
            response.success = False
            response.message = self.last_save_error or (
                "map saver is unavailable or already busy"
            )
        return response

    def _preview(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            (
                _robot_x,
                _robot_y,
                candidates,
                reachable_candidates,
            ) = self._find_candidates()
        except (RuntimeError, TransformException, ValueError) as error:
            response.success = False
            response.message = str(error)
            return response
        payload = {
            "candidate_count": len(candidates),
            "reachable_frontier_count": len(reachable_candidates),
            "map_quality": (
                None
                if self.latest_map_quality is None
                else self.latest_map_quality.as_dict()
            ),
            "map_age_seconds": round(max(0.0, self._map_age()), 2),
        }
        if candidates:
            payload["best"] = asdict(candidates[0])
        response.success = True
        response.message = json.dumps(payload, separators=(",", ":"))
        return response

    def _publish_zero(self) -> None:
        stop = Twist()
        for _ in range(5):
            self.stop_publisher.publish(stop)

    def _set_navigation_mode(self, enabled: bool) -> None:
        lease = Bool()
        lease.data = enabled
        for _ in range(3 if not enabled else 1):
            self.navigation_lease_publisher.publish(lease)

    def _record_travel(self, x: float, y: float) -> None:
        if self.last_travel_pose is not None:
            step = math.hypot(x - self.last_travel_pose[0], y - self.last_travel_pose[1])
            if step <= 0.5:
                self.travel_distance += step
        self.last_travel_pose = (x, y)

    def _on_emergency_stop(self, message: Bool) -> None:
        if message.data and (self.enabled or self.goal_handle is not None):
            self._stop_exploration("emergency stop received from command bridge")

    def _stop_exploration(self, reason: str) -> None:
        self.enabled = False
        self.state = "stopped"
        self.goal_generation += 1
        if self.goal_handle is not None and not self.goal_cancel_pending:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_request_pending = False
        self.goal_cancel_pending = False
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.goal_progress = None
        self.goal_candidate = None
        self.plan_generation += 1
        if self.plan_handle is not None:
            try:
                self.plan_handle.cancel_goal_async()
            except Exception:
                pass
        self.plan_handle = None
        self.plan_request_pending = False
        self.plan_candidate = None
        self.plan_context = None
        self._publish_zero()
        self._set_navigation_mode(False)
        self._publish_status(reason)

    def _prune_blacklist(
        self, now: float
    ) -> tuple[
        list[tuple[float, float]],
        list[tuple[float, float]],
        list[tuple[float, float]],
    ]:
        self.blacklist = [entry for entry in self.blacklist if entry[2] > now]
        self.staged_blacklist = [
            entry for entry in self.staged_blacklist if entry[2] > now
        ]
        return (
            [(entry[0], entry[1]) for entry in self.blacklist],
            [(entry[0], entry[1]) for entry in self.staged_blacklist],
            [
                (entry[0], entry[1])
                for entry in self.blacklist
                if not entry[3]
            ],
        )

    def _cancel_active_goal(self, reason: str) -> None:
        if self.goal_handle is None or self.goal_cancel_pending:
            return
        self.goal_cancel_pending = True
        self.goal_cancel_requested_at = self._seconds()
        self.goal_cancel_reason = reason
        self.state = "cancelling_goal"
        try:
            self.goal_handle.cancel_goal_async()
        except Exception as error:
            self._stop_exploration(f"{reason}; Nav2 cancel request failed: {error}")
            self._request_map_save("saving partial map after cancel failure")
            return
        self._publish_status(f"{reason}; cancelling active frontier goal")

    def _map_spec(self, message: OccupancyGrid) -> GridSpec:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return GridSpec(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=yaw,
        )

    def _find_candidates(self):
        if self.latest_map is None:
            raise RuntimeError("occupancy grid is not ready")
        robot_x, robot_y = self._robot_pose()
        spec = self._map_spec(self.latest_map)
        maximum_seed_distance = float(
            self.get_parameter("maximum_robot_free_seed_distance").value
        )
        reachable = reachable_free_cell_indices(
            self.latest_map.data,
            spec,
            robot_x=robot_x,
            robot_y=robot_y,
            maximum_seed_distance=maximum_seed_distance,
        )
        if not reachable:
            raise RuntimeError(
                "robot pose is not connected to nearby known free map space"
            )
        start_x, start_y = self.start_pose or (robot_x, robot_y)
        max_radius = float(self.get_parameter("maximum_exploration_radius").value)
        (
            active_blacklist,
            active_staged_blacklist,
            active_reached_blacklist,
        ) = self._prune_blacklist(self._seconds())

        def search(
            *,
            min_cells: int,
            min_distance: float,
            blacklisted: list[tuple[float, float]],
            staged_blacklisted: list[tuple[float, float]],
        ):
            found = frontier_candidates(
                self.latest_map.data,
                spec,
                robot_x=robot_x,
                robot_y=robot_y,
                min_cells=min_cells,
                min_distance=min_distance,
                max_distance=float(
                    self.get_parameter("maximum_goal_distance").value
                ),
                blacklisted=blacklisted,
                blacklist_radius=float(
                    self.get_parameter("blacklist_radius").value
                ),
                staged_blacklisted=staged_blacklisted,
                staged_blacklist_radius=float(
                    self.get_parameter("staged_goal_blacklist_radius").value
                ),
                goal_standoff=float(
                    self.get_parameter("frontier_goal_standoff").value
                ),
                maximum_goal_step_distance=float(
                    self.get_parameter("maximum_goal_step_distance").value
                ),
                minimum_obstacle_clearance=float(
                    self.get_parameter("minimum_goal_obstacle_clearance").value
                ),
                maximum_robot_free_seed_distance=maximum_seed_distance,
            )
            return [
                candidate
                for candidate in found
                if max_radius <= 0.0
                or math.hypot(candidate.x - start_x, candidate.y - start_y)
                <= max_radius
            ]

        normal_min_length = float(
            self.get_parameter("minimum_frontier_length").value
        )
        normal_min_cells = max(2, math.ceil(normal_min_length / spec.resolution))
        candidates = search(
            min_cells=normal_min_cells,
            min_distance=float(self.get_parameter("minimum_goal_distance").value),
            blacklisted=active_blacklist,
            staged_blacklisted=active_staged_blacklist,
        )

        # Completion uses a more sensitive search than ordinary goal scoring.
        # This prevents a small but navigable gray boundary from being treated
        # as finished merely because it is below the preferred frontier size.
        completion_min_length = float(
            self.get_parameter("minimum_completion_frontier_length").value
        )
        completion_min_cells = max(
            2, math.ceil(completion_min_length / spec.resolution)
        )
        completion_min_distance = float(
            self.get_parameter("minimum_completion_goal_distance").value
        )
        reachable_candidates = search(
            min_cells=completion_min_cells,
            min_distance=completion_min_distance,
            # Failed plans remain completion blockers because costmaps can
            # settle. Reached boundaries do not: they were already explored
            # and must not become destinations again. frontier_candidates
            # compares this list only with the final frontier coordinate, so
            # a path to new space may still traverse the old region.
            blacklisted=active_reached_blacklist,
            staged_blacklisted=[],
        )
        if not candidates:
            candidates = search(
                min_cells=completion_min_cells,
                min_distance=completion_min_distance,
                blacklisted=active_blacklist,
                staged_blacklisted=active_staged_blacklist,
            )

        self.active_frontier_count = len(candidates)
        self.reachable_frontier_count = len(reachable_candidates)
        return robot_x, robot_y, candidates, reachable_candidates

    def _tick(self) -> None:
        if self.save_requested_reason and not self.save_pending:
            pending_reason = self.save_requested_reason
            self._request_map_save(pending_reason)
        if not self.enabled:
            return
        now = self._seconds()
        minimum_battery = int(
            self.get_parameter("minimum_battery_percent").value
        )
        if (
            self.latest_battery_percent is not None
            and self.latest_battery_percent <= minimum_battery
        ):
            battery = self.latest_battery_percent
            self._stop_exploration(
                f"battery reached safety threshold ({battery}% <= {minimum_battery}%)"
            )
            self._request_map_save("saving partial map before low-battery shutdown")
            return
        elapsed = now - self.started_at
        maximum_runtime = float(self.get_parameter("maximum_runtime").value)
        if maximum_runtime > 0.0 and elapsed >= maximum_runtime:
            self._stop_exploration("maximum mapping time reached")
            self._request_map_save("saving partial map after time limit")
            return

        maximum_map_age = float(self.get_parameter("maximum_map_age").value)
        map_age = self._map_age(now)
        if maximum_map_age > 0.0 and map_age > maximum_map_age:
            self._stop_exploration(
                f"occupancy grid stopped updating ({map_age:.1f} s old)"
            )
            self._request_map_save("saving partial map after map stream failure")
            return

        # Account for motion while an action goal is active. Previously travel
        # was sampled only between goals, so a successful one-metre drive could
        # appear as zero progress and block the completion guard forever.
        try:
            travel_x, travel_y = self._robot_pose()
            self._record_travel(travel_x, travel_y)
        except TransformException:
            pass

        if self.goal_handle is not None:
            if self.goal_cancel_pending:
                cancel_timeout = float(
                    self.get_parameter("goal_cancel_timeout").value
                )
                if (
                    cancel_timeout > 0.0
                    and now - self.goal_cancel_requested_at >= cancel_timeout
                ):
                    reason = self.goal_cancel_reason or "frontier goal cancellation"
                    self._stop_exploration(
                        f"{reason}; Nav2 did not acknowledge cancellation"
                    )
                    self._request_map_save(
                        "saving partial map after Nav2 cancellation timeout"
                    )
                return

            progress_timeout = float(
                self.get_parameter("goal_progress_timeout").value
            )
            if self.goal_progress is not None and self.goal_progress.stalled(
                now=now, timeout=progress_timeout
            ):
                self._cancel_active_goal(
                    f"frontier made no progress for {progress_timeout:.1f} s"
                )
                return
            timeout = float(self.get_parameter("goal_timeout").value)
            if timeout > 0.0 and now - self.goal_sent_at >= timeout:
                self.get_logger().warn("frontier goal timed out; cancelling")
                self._cancel_active_goal(
                    f"frontier goal exceeded {timeout:.1f} s deadline"
                )
            return
        if self.goal_request_pending:
            return
        if self.plan_request_pending:
            plan_timeout = float(
                self.get_parameter("frontier_plan_timeout").value
            )
            if plan_timeout > 0.0 and now - self.plan_sent_at >= plan_timeout:
                if self.plan_handle is not None:
                    try:
                        self.plan_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._finish_plan_failure(
                    f"frontier path check exceeded {plan_timeout:.1f} s"
                )
            return
        if now < self.next_goal_not_before:
            self.state = "goal_retry_cooldown"
            return
        if elapsed < float(self.get_parameter("startup_delay").value):
            return
        if self.latest_map is None:
            self.state = "waiting_for_map"
            self._publish_status("waiting for occupancy grid")
            return
        if not self.navigator.server_is_ready():
            self.state = "waiting_for_nav2"
            self._publish_status("waiting for Nav2 action server")
            return
        if not self.path_planner.server_is_ready():
            self.state = "waiting_for_path_planner"
            self._publish_status("waiting for Nav2 path planner")
            return

        try:
            (
                robot_x,
                robot_y,
                candidates,
                reachable_candidates,
            ) = self._find_candidates()
        except RuntimeError as error:
            self.state = "waiting_for_map_alignment"
            self._publish_status(f"waiting for safe map alignment: {error}")
            return
        except (TransformException, ValueError) as error:
            self.state = "waiting_for_tf"
            self._publish_status(f"waiting for robot pose: {error}")
            return
        if self.start_pose is None:
            self.start_pose = (robot_x, robot_y)

        if not candidates and reachable_candidates:
            # A reached frontier can temporarily disappear from the active
            # list during its short cooldown. It is not map completion: keep
            # checking until it changes or becomes actionable again.
            self.empty_map_count = 0
            self.last_empty_map_sequence = -1
            self.state = "waiting_for_frontier_retry"
            retry_seconds = 0.0
            failed_frontier_expiries = [
                entry[2] for entry in self.blacklist if entry[3]
            ]
            if failed_frontier_expiries:
                retry_seconds = max(
                    0.0, min(failed_frontier_expiries) - now
                )
            if self.staged_blacklist:
                retry_seconds = min(
                    retry_seconds or float("inf"),
                    max(
                        0.0,
                        min(entry[2] for entry in self.staged_blacklist) - now,
                    ),
                )
            self._publish_status(
                f"{len(reachable_candidates)} reachable frontier(s) remain; "
                f"retrying after blacklist ({retry_seconds:.1f} s)"
            )
            return

        if not candidates:
            if self.last_empty_map_sequence != self.map_sequence:
                self.empty_map_count += 1
                self.last_empty_map_sequence = self.map_sequence
            self.state = "checking_completion"
            minimum_runtime = float(self.get_parameter("minimum_runtime").value)
            minimum_travel = float(
                self.get_parameter("minimum_completion_travel_distance").value
            )
            stable_maps = int(self.get_parameter("completion_stable_maps").value)
            if (
                elapsed >= minimum_runtime
                and self.empty_map_count >= stable_maps
                and self.travel_distance >= minimum_travel
            ):
                quality_errors = self._save_quality_errors()
                if quality_errors:
                    message = (
                        "exploration ended without a publishable map: "
                        + "; ".join(quality_errors)
                    )
                    self._stop_exploration(message)
                    self.state = "map_quality_failed"
                    self._publish_status(message)
                    return
                self.enabled = False
                self.state = "completed"
                self._publish_zero()
                self._set_navigation_mode(False)
                self._publish_status("exploration complete")
                self._request_map_save("saving completed map")
            else:
                self._publish_status(
                    "no safely reachable gray boundary; checking completion "
                    f"(travel={self.travel_distance:.2f}/{minimum_travel:.2f} m)"
                )
            return

        self.empty_map_count = 0
        self.last_empty_map_sequence = -1
        self._check_frontier_path(
            candidates[0], robot_x, robot_y, len(candidates)
        )

    def _candidate_pose(self, candidate, robot_x: float, robot_y: float):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = candidate.x
        pose.pose.position.y = candidate.y
        yaw = math.atan2(candidate.y - robot_y, candidate.x - robot_x)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _check_frontier_path(
        self, candidate, robot_x: float, robot_y: float, count: int
    ) -> None:
        """Ask Nav2 whether its real costmap can reach a frontier goal."""

        goal = ComputePathToPose.Goal()
        goal.goal = self._candidate_pose(candidate, robot_x, robot_y)
        goal.planner_id = "GridBased"
        goal.use_start = False
        self.plan_candidate = candidate
        self.plan_context = (robot_x, robot_y, count)
        self.plan_request_pending = True
        self.plan_handle = None
        self.plan_generation += 1
        generation = self.plan_generation
        self.plan_sent_at = self._seconds()
        self.state = "checking_frontier_path"
        self._publish_status(
            f"checking Nav2 path to best of {count} frontier candidates"
        )
        try:
            future = self.path_planner.send_goal_async(goal)
        except Exception as error:
            self._finish_plan_failure(f"frontier path check could not start: {error}")
            return
        future.add_done_callback(
            lambda completed, plan_generation=generation: self._on_plan_response(
                completed, plan_generation
            )
        )

    def _on_plan_response(self, future, generation: int) -> None:
        if generation != self.plan_generation:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            handle = future.result()
        except Exception as error:
            self._finish_plan_failure(f"frontier path check failed: {error}")
            return
        if not handle.accepted:
            self._finish_plan_failure("frontier path check was rejected")
            return
        self.plan_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, plan_generation=generation: self._on_plan_result(
                completed, plan_generation
            )
        )

    def _on_plan_result(self, future, generation: int) -> None:
        if generation != self.plan_generation:
            return
        try:
            wrapped_result = future.result()
            succeeded = int(wrapped_result.status) == GoalStatus.STATUS_SUCCEEDED
            has_path = bool(wrapped_result.result.path.poses)
        except Exception as error:
            self._finish_plan_failure(f"frontier path result failed: {error}")
            return
        if not succeeded or not has_path:
            self._finish_plan_failure("Nav2 found no safe path to frontier")
            return

        candidate = self.plan_candidate
        context = self.plan_context
        self.plan_handle = None
        self.plan_request_pending = False
        self.plan_candidate = None
        self.plan_context = None
        if not self.enabled or candidate is None or context is None:
            return
        self._send_goal(candidate, context[0], context[1], context[2])

    def _finish_plan_failure(self, message: str) -> None:
        candidate = self.plan_candidate
        self.plan_generation += 1
        self.plan_handle = None
        self.plan_request_pending = False
        self.plan_candidate = None
        self.plan_context = None
        if candidate is not None:
            self._blacklist_candidate(candidate, True)
        if not self.enabled:
            return
        self.next_goal_not_before = self._seconds() + min(
            1.0, float(self.get_parameter("goal_retry_delay").value)
        )
        self.state = "frontier_inaccessible"
        self._publish_status(f"{message}; checking another gray boundary")

    def _send_goal(self, candidate, robot_x: float, robot_y: float, count: int) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = self._candidate_pose(candidate, robot_x, robot_y)
        self.goal_candidate = candidate
        self.goal_request_pending = True
        self.goal_cancel_pending = False
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.goal_generation += 1
        generation = self.goal_generation
        self.goal_sent_at = self._seconds()
        self.goal_progress = GoalProgress.started(self.goal_sent_at)
        self.distance_remaining = candidate.distance
        self.state = "sending_goal"
        self.goal_publisher.publish(goal.pose)
        self._publish_status(f"selected frontier from {count} candidates")
        try:
            future = self.navigator.send_goal_async(
                goal,
                feedback_callback=lambda feedback, goal_generation=generation: (
                    self._on_feedback(feedback, goal_generation)
                ),
            )
        except Exception as error:
            self.goal_request_pending = False
            self._finish_goal_failure(f"goal request could not be sent: {error}")
            return
        future.add_done_callback(
            lambda completed, goal_generation=generation: self._on_goal_response(
                completed, goal_generation
            )
        )

    def _on_goal_response(self, future, generation: int) -> None:
        if generation != self.goal_generation:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as error:  # rclpy future transports exceptions here
            self._finish_goal_failure(f"goal request failed: {error}")
            return
        if not handle.accepted:
            self._finish_goal_failure("frontier goal rejected")
            return
        self.goal_handle = handle
        if not self.enabled:
            self.goal_cancel_pending = True
            handle.cancel_goal_async()
            return
        self.state = "navigating"
        self._publish_status("frontier goal accepted")
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_generation=generation: self._on_goal_result(
                completed, goal_generation
            )
        )

    def _on_feedback(self, feedback_message, generation: int) -> None:
        if generation != self.goal_generation or self.goal_cancel_pending:
            return
        distance = float(feedback_message.feedback.distance_remaining)
        if not math.isfinite(distance) or distance < 0.0:
            return
        self.distance_remaining = distance
        if self.goal_progress is not None:
            self.goal_progress.update(
                distance,
                now=self._seconds(),
                minimum_delta=float(
                    self.get_parameter("goal_progress_minimum_delta").value
                ),
            )

    def _blacklist_candidate(self, candidate, failed: bool) -> None:
        if candidate is None:
            return
        lifetime_name = (
            "failed_goal_blacklist_seconds"
            if failed
            else "reached_goal_blacklist_seconds"
        )
        expires = self._seconds() + float(self.get_parameter(lifetime_name).value)
        self.blacklist.append(
            (candidate.frontier_x, candidate.frontier_y, expires, failed)
        )
        if failed:
            self.staged_blacklist.append((candidate.x, candidate.y, expires))

    def _blacklist_current(self, failed: bool) -> None:
        if self.goal_candidate is None:
            return
        self._blacklist_candidate(self.goal_candidate, failed)
        self.goal_candidate = None

    def _finish_goal_failure(self, message: str) -> None:
        """Blacklist one bad region and stop safely after repeated failures."""

        self._blacklist_current(True)
        self.distance_remaining = None
        self.goal_cancel_pending = False
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.goal_progress = None
        if not self.enabled:
            return
        self.consecutive_goal_failures += 1
        maximum_failures = int(
            self.get_parameter("maximum_consecutive_goal_failures").value
        )
        if maximum_failures > 0 and self.consecutive_goal_failures >= maximum_failures:
            failure_message = (
                f"{message}; stopped after {self.consecutive_goal_failures} "
                "consecutive frontier failures"
            )
            self._stop_exploration(failure_message)
            self._request_map_save("saving partial map after navigation failures")
            return
        self.next_goal_not_before = self._seconds() + float(
            self.get_parameter("goal_retry_delay").value
        )
        self.state = "goal_failed"
        self._publish_status(
            f"{message}; retry {self.consecutive_goal_failures}/{maximum_failures} "
            "after cooldown"
        )

    def _on_goal_result(self, future, generation: int) -> None:
        if generation != self.goal_generation:
            return
        self.goal_handle = None
        self.goal_cancel_pending = False
        if not self.enabled:
            self.goal_candidate = None
            self.distance_remaining = None
            return
        try:
            status = int(future.result().status)
        except Exception as error:
            self._finish_goal_failure(f"goal result failed: {error}")
            return
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if not succeeded:
            self._finish_goal_failure(f"frontier failed with status={status}")
            return
        self._blacklist_current(False)
        self.distance_remaining = None
        self.goal_progress = None
        self.goal_cancel_requested_at = 0.0
        self.goal_cancel_reason = ""
        self.consecutive_goal_failures = 0
        self.next_goal_not_before = 0.0
        self.state = "goal_reached"
        self._publish_status("frontier reached")

    def _request_map_save(self, reason: str) -> bool:
        if self.save_pending:
            self.last_save_error = "a map save is already in progress"
            return False
        quality_errors = self._save_quality_errors()
        if quality_errors:
            self.save_requested_reason = ""
            self.last_save_error = "map quality check failed: " + "; ".join(
                quality_errors
            )
            self._publish_status(self.last_save_error)
            return False
        if not self.map_saver.service_is_ready():
            self.save_requested_reason = reason
            self.last_save_error = "map saver service is not ready; save queued"
            self._publish_status("map saver unavailable; save queued")
            return True
        self.save_requested_reason = ""
        self.last_save_error = ""
        request = SaveMap.Request()
        map_output = str(self.get_parameter("map_output").value)
        self.save_attempt_sequence += 1
        staging_output = (
            f"{map_output}.pending-{self.save_attempt_sequence}-{int(self._seconds() * 1000)}"
        )
        cleanup_saved_map(staging_output)
        request.map_topic = str(self.get_parameter("map_topic").value)
        request.map_url = staging_output
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self.save_pending = True
        self.saving_map_output = map_output
        self.saving_staging_output = staging_output
        try:
            x, y, yaw = self._robot_pose_full()
            self.saving_robot_pose = {"x": x, "y": y, "yaw": yaw}
        except TransformException as error:
            self.saving_robot_pose = None
            self.get_logger().warn(
                f"map will be saved without a robot pose: {error}"
            )
        self._publish_status(reason)
        try:
            future = self.map_saver.call_async(request)
        except Exception as error:
            self.save_pending = False
            cleanup_saved_map(staging_output)
            self.last_save_error = f"map save request could not be sent: {error}"
            self._publish_status(self.last_save_error)
            return False
        future.add_done_callback(self._on_map_saved)
        return True

    def _on_map_saved(self, future) -> None:
        self.save_pending = False
        try:
            saved = bool(future.result().result)
        except Exception as error:
            saved = False
            self.last_save_error = f"map save failed: {error}"
        if saved:
            try:
                promote_saved_map(
                    self.saving_staging_output,
                    self.saving_map_output,
                )
            except (OSError, ValueError) as error:
                saved = False
                self.last_save_error = (
                    "map staging validation/promotion failed: " + str(error)
                )
        if not saved:
            cleanup_saved_map(self.saving_staging_output)
            if not self.last_save_error:
                self.last_save_error = "map saver reported failure"
            self._publish_status(self.last_save_error)
            return

        self.last_save_error = ""
        self.saved_map = self.saving_map_output
        self.saved_pose = self._persist_saved_mapping_pose(
            self.saving_map_output,
            self.saving_robot_pose,
        )
        self.save_sequence += 1
        self.pose_graph_save_requested = True
        self.pose_graph_saved = False
        self._request_pose_graph_save()
        self._publish_status(f"map saved and validated: {self.saving_map_output}")

    def _persist_saved_mapping_pose(self, map_output: str, pose):
        """Bind the final SLAM pose to the exact raw PGM checksum."""

        if not isinstance(pose, dict):
            return None
        image_path = Path(f"{map_output}.pgm")
        pose_path = Path(f"{map_output}_pose.json")
        try:
            image_sha256 = hashlib.sha256(image_path.read_bytes()).hexdigest()
            payload = {
                "schema_version": 1,
                "map_sha256": image_sha256,
                "frame_id": self.map_frame,
                "x": round(float(pose["x"]), 6),
                "y": round(float(pose["y"]), 6),
                "yaw": round(float(pose["yaw"]), 6),
                "saved_unix": self._seconds(),
            }
            temporary = pose_path.with_name(
                f".{pose_path.name}.tmp-{os.getpid()}"
            )
            temporary.write_text(
                json.dumps(payload, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, pose_path)
            return payload
        except (OSError, KeyError, TypeError, ValueError) as error:
            self.get_logger().warn(f"saved-map robot pose could not be persisted: {error}")
            return None

    def _publish_periodic_status(self) -> None:
        if self.enabled:
            self._set_navigation_mode(True)
            self._publish_status(self.status_message, log=False)

    def _publish_status(self, message: str, *, log: bool = True) -> None:
        self.status_message = message
        payload = {
            "state": self.state,
            "enabled": self.enabled,
            "message": message,
            "map_sequence": self.map_sequence,
            "empty_map_count": self.empty_map_count,
            "active_frontier_count": self.active_frontier_count,
            "reachable_frontier_count": self.reachable_frontier_count,
            "distance_remaining": self.distance_remaining,
            "saved_map": self.saved_map,
            "saved_pose": self.saved_pose,
            "save_sequence": self.save_sequence,
            "save_pending": self.save_pending,
            "battery_percent": self.latest_battery_percent,
            "minimum_battery_percent": int(
                self.get_parameter("minimum_battery_percent").value
            ),
            "pose_graph_resume_state": self.pose_graph_resume_state,
            "pose_graph_resumed": self.pose_graph_resumed,
            "pose_graph_save_pending": self.pose_graph_save_pending,
            "pose_graph_saved": self.pose_graph_saved,
            "pose_graph_error": self.pose_graph_last_error or None,
            "travel_distance": round(self.travel_distance, 3),
            "consecutive_goal_failures": self.consecutive_goal_failures,
            "goal_cancel_pending": self.goal_cancel_pending,
            "path_check_pending": self.plan_request_pending,
            "map_age_seconds": (
                None
                if self.last_map_received_at <= 0.0
                else round(max(0.0, self._map_age()), 2)
            ),
            "map_quality": (
                None
                if self.latest_map_quality is None
                else self.latest_map_quality.as_dict()
            ),
            "last_save_error": self.last_save_error or None,
        }
        if self.goal_candidate is not None:
            payload["goal"] = asdict(self.goal_candidate)
        if self.plan_candidate is not None:
            payload["path_check_goal"] = asdict(self.plan_candidate)
        status = String()
        status.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(status)
        if log:
            self.get_logger().info(message)


def main() -> None:
    rclpy.init()
    node = AutonomousMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_exploration("process interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
