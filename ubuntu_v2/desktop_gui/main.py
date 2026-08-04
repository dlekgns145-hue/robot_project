#!/usr/bin/env python3
"""Windows/macOS desktop control center for Robot Control v2."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

from PySide6.QtCore import QEvent, QProcess, QSettings, QTimer, Qt
from PySide6.QtGui import QCloseEvent, QFont, QImage, QKeyEvent, QPixmap
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from control_logic import FollowSettings
from robot_client import RobotClient
from theme import APP_STYLESHEET
from vision_worker import VisionWorker


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Control v2")
        self.resize(1380, 900)
        self.setMinimumSize(960, 640)
        self.settings_store = QSettings("robot-project", "robot-control-v2")
        self.client: RobotClient | None = None
        self.vision: VisionWorker | None = None
        self.gateway_connected = False
        self.robot_connected = False
        self.current_robot_ip = ""
        self._last_robot_state: tuple[bool, str] | None = None
        self.follow_active = False
        self.vision_mode: str | None = None
        self.navigation_process: QProcess | None = None
        self._manual_keys: set[int] = set()
        self._build_ui()
        self._load_settings()
        QTimer.singleShot(400, self._auto_connect)

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("AppRoot")
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(22, 20, 22, 22)
        root_layout.setSpacing(18)

        header = QFrame()
        header.setObjectName("HeaderPanel")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(22, 15, 18, 15)

        brand = QVBoxLayout()
        brand.setSpacing(2)
        eyebrow = QLabel("ROBOT OPERATIONS")
        eyebrow.setObjectName("Eyebrow")
        title = QLabel("Control Center")
        title.setObjectName("AppTitle")
        subtitle = QLabel("실시간 비전 · 자율주행 · 원격 제어")
        subtitle.setObjectName("Subtitle")
        brand.addWidget(eyebrow)
        brand.addWidget(title)
        brand.addWidget(subtitle)
        header_layout.addLayout(brand)
        header_layout.addStretch(1)

        self.connection_label = QLabel("●  연결 안 됨")
        self.connection_label.setObjectName("ConnectionStatus")
        self.connection_label.setProperty("state", "offline")
        self.connection_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_layout.addWidget(self.connection_label)
        root_layout.addWidget(header)

        body = QHBoxLayout()
        body.setSpacing(18)
        root_layout.addLayout(body, 1)

        left = QVBoxLayout()
        left.setSpacing(14)
        body.addLayout(left, 1)

        preview_card = QFrame()
        preview_card.setObjectName("PreviewCard")
        preview_layout = QVBoxLayout(preview_card)
        preview_layout.setContentsMargins(16, 14, 16, 16)
        preview_layout.setSpacing(10)
        preview_header = QHBoxLayout()
        preview_title = QLabel("LIVE VISION")
        preview_title.setObjectName("SectionKicker")
        self.metrics_label = QLabel("영상 대기 중")
        self.metrics_label.setObjectName("VisionMetrics")
        preview_hint = QLabel("ROBOT CAMERA")
        preview_hint.setObjectName("CameraBadge")
        preview_header.addWidget(preview_title)
        preview_header.addStretch(1)
        preview_header.addWidget(self.metrics_label)
        preview_header.addWidget(preview_hint)
        preview_layout.addLayout(preview_header)

        self.video = QLabel(
            "카메라 스트림 대기 중\nPerception 또는 Follow Me를 시작하세요"
        )
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # Keep the preview responsive on shorter laptop displays. A large fixed
        # minimum caused the metrics strip to paint over the camera image.
        self.video.setMinimumSize(400, 140)
        self.video.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self.video.setObjectName("VideoSurface")
        preview_layout.addWidget(self.video, 1)
        left.addWidget(preview_card, 1)

        telemetry_group = QGroupBox("실시간 텔레메트리")
        telemetry = QGridLayout(telemetry_group)
        telemetry.setHorizontalSpacing(10)
        telemetry.setVerticalSpacing(10)
        self.robot_address_label = QLabel("-")
        self.lidar_label = QLabel("-")
        self.distance_label = QLabel("-")
        self.avoid_label = QLabel("-")
        self.applied_label = QLabel("-")
        telemetry.addWidget(
            self._metric_card("ROBOT", self.robot_address_label), 0, 0, 1, 2
        )
        telemetry.addWidget(
            self._metric_card("COMMAND", self.applied_label), 0, 2
        )
        telemetry.addWidget(self._metric_card("LIDAR", self.lidar_label), 1, 0)
        telemetry.addWidget(
            self._metric_card("FRONT", self.distance_label), 1, 1
        )
        telemetry.addWidget(self._metric_card("AVOID", self.avoid_label), 1, 2)
        left.addWidget(telemetry_group)

        log_group = QGroupBox("시스템 로그")
        log_layout = QVBoxLayout(log_group)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        self.log.setMinimumHeight(60)
        self.log.setMaximumHeight(100)
        log_layout.addWidget(self.log)
        left.addWidget(log_group)

        right_scroll = QScrollArea()
        right_scroll.setObjectName("ControlScroll")
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.Shape.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        right_scroll.setMinimumWidth(410)
        right_scroll.setMaximumWidth(480)
        right_column = QVBoxLayout()
        right_column.setSpacing(12)
        right_column.addWidget(right_scroll, 1)
        body.addLayout(right_column)
        right_panel = QWidget()
        right_panel.setObjectName("ControlPanel")
        right = QVBoxLayout(right_panel)
        right.setContentsMargins(0, 0, 4, 0)
        right.setSpacing(14)
        right_scroll.setWidget(right_panel)

        connection_group = QGroupBox("연결 설정")
        connection_form = QFormLayout(connection_group)
        connection_form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow
        )
        self.host_edit = QLineEdit()
        self.host_edit.setPlaceholderText("예: 192.168.64.15")
        self.command_port = QSpinBox()
        self.command_port.setRange(1, 65535)
        self.robot_host_edit = QLineEdit()
        self.robot_host_edit.setPlaceholderText("raspberrypi.local")
        self.token_edit = QLineEdit()
        self.token_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.token_edit.setPlaceholderText("저장된 제어 토큰")
        self.connect_button = QPushButton("서버에 연결")
        self.connect_button.setProperty("role", "primary")
        self.connect_button.setMinimumHeight(42)
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_form.addRow("Ubuntu VM IP", self.host_edit)
        connection_form.addRow("명령 포트", self.command_port)
        connection_form.addRow("로봇 주소", self.robot_host_edit)
        connection_form.addRow("제어 토큰", self.token_edit)
        connection_form.addRow(self.connect_button)
        right.addWidget(connection_group)

        follow_group = QGroupBox("자동화 및 비전")
        follow_form = QFormLayout(follow_group)
        follow_form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow
        )
        self.camera_source = QComboBox()
        self.camera_source.addItem("로봇 카메라 (자동)", "robot")
        self.camera_source.addItem("이 컴퓨터의 카메라", "local")
        self.camera_source.addItem("네트워크 영상 URL", "network")
        self.camera_source.currentIndexChanged.connect(self._update_camera_fields)
        self.camera_index = QSpinBox()
        self.camera_index.setRange(0, 10)
        self.camera_url = QLineEdit()
        self.camera_url.setPlaceholderText("예: http://robot-ip:8080/stream.mjpg")
        model_row = QHBoxLayout()
        self.model_edit = QLineEdit()
        browse_button = QPushButton("찾기")
        browse_button.setProperty("role", "quiet")
        browse_button.clicked.connect(self.choose_model)
        model_row.addWidget(self.model_edit, 1)
        model_row.addWidget(browse_button)
        self.linear_speed = self._double_spin(0.0, 0.5, 0.35, 0.01)
        self.angular_speed = self._double_spin(0.0, 0.8, 0.4, 0.01)
        self.stop_ratio = self._double_spin(0.2, 0.9, 0.55, 0.01)
        self.frame_skip = QSpinBox()
        self.frame_skip.setRange(1, 10)
        self.frame_skip.setValue(2)
        self.nav_x = self._double_spin(-100.0, 100.0, 1.0, 0.1)
        self.nav_y = self._double_spin(-100.0, 100.0, 0.5, 0.1)
        self.nav_yaw = self._double_spin(-3.14, 3.14, 0.0, 0.1)
        feature_buttons = QGridLayout()
        self.perception_button = QPushButton("비전 인식 시작")
        self.perception_button.setProperty("role", "primary")
        self.perception_button.setCheckable(True)
        self.perception_button.clicked.connect(self.toggle_perception)
        self.follow_button = QPushButton("Follow Me")
        self.follow_button.setProperty("role", "primary")
        self.follow_button.setCheckable(True)
        self.follow_button.clicked.connect(self.toggle_follow)
        self.navigation_button = QPushButton("Navigation")
        self.navigation_button.setProperty("role", "secondary")
        self.navigation_button.setCheckable(True)
        self.navigation_button.clicked.connect(self.toggle_navigation)
        stop_features = QPushButton("전체 중지")
        stop_features.setProperty("role", "quiet")
        stop_features.clicked.connect(self.stop_all_features)
        for button in (
            self.perception_button,
            self.follow_button,
            self.navigation_button,
            stop_features,
        ):
            button.setMinimumHeight(40)
        feature_buttons.addWidget(self.perception_button, 0, 0)
        feature_buttons.addWidget(self.follow_button, 0, 1)
        feature_buttons.addWidget(self.navigation_button, 1, 0)
        feature_buttons.addWidget(stop_features, 1, 1)
        follow_form.addRow(feature_buttons)
        follow_form.addRow("카메라 입력", self.camera_source)
        follow_form.addRow("카메라 번호", self.camera_index)
        follow_form.addRow("영상 URL", self.camera_url)
        follow_form.addRow("YOLO 모델", model_row)
        follow_form.addRow("전진 속도", self.linear_speed)
        follow_form.addRow("회전 속도", self.angular_speed)
        follow_form.addRow("정지 감도", self.stop_ratio)
        follow_form.addRow("추론 간격", self.frame_skip)
        navigation_title = QLabel("NAVIGATION GOAL")
        navigation_title.setObjectName("SubsectionLabel")
        follow_form.addRow(navigation_title)
        follow_form.addRow("X", self.nav_x)
        follow_form.addRow("Y", self.nav_y)
        follow_form.addRow("Yaw", self.nav_yaw)
        right.addWidget(follow_group)

        manual_group = QGroupBox("수동 주행")
        manual = QGridLayout(manual_group)
        manual.setSpacing(8)
        forward = QPushButton("▲\n전진")
        left_turn = QPushButton("◀\n좌회전")
        stop = QPushButton("■\n정지")
        right_turn = QPushButton("▶\n우회전")
        backward = QPushButton("▼\n후진")
        for button in (forward, left_turn, stop, right_turn, backward):
            button.setProperty("role", "drive")
            button.setMinimumHeight(54)
        stop.setProperty("role", "driveStop")
        manual.addWidget(forward, 0, 1)
        manual.addWidget(left_turn, 1, 0)
        manual.addWidget(stop, 1, 1)
        manual.addWidget(right_turn, 1, 2)
        manual.addWidget(backward, 2, 1)
        keyboard_hint = QLabel("키보드  ↑ ↓ ← →  또는  W A S D   ·   Space 정지")
        keyboard_hint.setObjectName("ControlHint")
        keyboard_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        manual.addWidget(keyboard_hint, 3, 0, 1, 3)
        self._bind_hold_button(forward, 1.0, 0.0)
        self._bind_hold_button(backward, -0.6, 0.0)
        self._bind_hold_button(left_turn, 0.0, 1.0)
        self._bind_hold_button(right_turn, 0.0, -1.0)
        stop.clicked.connect(self.stop_motion)
        right.insertWidget(1, manual_group)

        self.emergency_button = QPushButton("긴급 정지   EMERGENCY STOP")
        self.emergency_button.setObjectName("EmergencyButton")
        self.emergency_button.setMinimumHeight(58)
        self.emergency_button.clicked.connect(self.emergency_stop)
        right_column.addWidget(self.emergency_button)
        right.addStretch(1)

    @staticmethod
    def _metric_card(label: str, value: QLabel) -> QFrame:
        card = QFrame()
        card.setProperty("card", "metric")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 11)
        layout.setSpacing(3)
        caption = QLabel(label)
        caption.setObjectName("MetricCaption")
        value.setObjectName("MetricValue")
        value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(caption)
        layout.addWidget(value)
        return card

    @staticmethod
    def _double_spin(
        minimum: float, maximum: float, value: float, step: float
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setValue(value)
        widget.setSingleStep(step)
        widget.setDecimals(2)
        return widget

    def _bind_hold_button(
        self, button: QPushButton, linear_scale: float, angular_scale: float
    ) -> None:
        button.pressed.connect(lambda: self.manual_motion(linear_scale, angular_scale))
        button.released.connect(self.stop_motion)

    @staticmethod
    def _direction_for_key(key: int) -> str | None:
        mapping = {
            int(Qt.Key.Key_Up): "forward",
            int(Qt.Key.Key_W): "forward",
            int(Qt.Key.Key_Down): "backward",
            int(Qt.Key.Key_S): "backward",
            int(Qt.Key.Key_Left): "left",
            int(Qt.Key.Key_A): "left",
            int(Qt.Key.Key_Right): "right",
            int(Qt.Key.Key_D): "right",
        }
        return mapping.get(key)

    def _keyboard_control_blocked(self) -> bool:
        focus = QApplication.focusWidget()
        return isinstance(
            focus,
            (QLineEdit, QAbstractSpinBox, QComboBox, QPlainTextEdit),
        )

    def _apply_keyboard_motion(self) -> None:
        directions = {
            direction
            for key in self._manual_keys
            if (direction := self._direction_for_key(key)) is not None
        }
        linear_scale = 0.0
        if "forward" in directions and "backward" not in directions:
            linear_scale = 1.0
        elif "backward" in directions and "forward" not in directions:
            linear_scale = -0.6

        angular_scale = 0.0
        if "left" in directions and "right" not in directions:
            angular_scale = 1.0
        elif "right" in directions and "left" not in directions:
            angular_scale = -1.0

        if linear_scale == 0.0 and angular_scale == 0.0:
            self.stop_motion()
        else:
            self.manual_motion(linear_scale, angular_scale)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        key = int(event.key())
        direction = self._direction_for_key(key)
        if key == int(Qt.Key.Key_Space) and not self._keyboard_control_blocked():
            self._manual_keys.clear()
            self.stop_motion()
            event.accept()
            return
        if direction is None or self._keyboard_control_blocked():
            super().keyPressEvent(event)
            return
        if not event.isAutoRepeat():
            self._manual_keys.add(key)
            self._apply_keyboard_motion()
        event.accept()

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        key = int(event.key())
        if self._direction_for_key(key) is None or key not in self._manual_keys:
            super().keyReleaseEvent(event)
            return
        if not event.isAutoRepeat():
            self._manual_keys.discard(key)
            self._apply_keyboard_motion()
        event.accept()

    def event(self, event: QEvent) -> bool:
        manual_keys = getattr(self, "_manual_keys", None)
        if event.type() == QEvent.Type.WindowDeactivate and manual_keys:
            manual_keys.clear()
            self.stop_motion()
        return super().event(event)

    def _load_settings(self) -> None:
        self.host_edit.setText(self.settings_store.value("host", ""))
        self.command_port.setValue(int(self.settings_store.value("command_port", 9999)))
        self.robot_host_edit.setText(
            self.settings_store.value("robot_host", "raspberrypi.local")
        )
        self.token_edit.setText(self.settings_store.value("token", ""))
        source = self.settings_store.value("camera_source", "robot")
        saved_camera_url = self.settings_store.value("camera_url", "")
        # Migrate the old broken state where "network" was saved with no URL.
        if source == "network" and not str(saved_camera_url).strip():
            source = "robot"
        source_index = self.camera_source.findData(source)
        self.camera_source.setCurrentIndex(max(source_index, 0))
        self.camera_index.setValue(int(self.settings_store.value("camera_index", 0)))
        self.camera_url.setText(saved_camera_url)
        self.nav_x.setValue(float(self.settings_store.value("nav_x", 1.0)))
        self.nav_y.setValue(float(self.settings_store.value("nav_y", 0.5)))
        self.nav_yaw.setValue(float(self.settings_store.value("nav_yaw", 0.0)))
        self._update_camera_fields()
        default_model = Path(__file__).resolve().parents[2] / "yolov8n-pose.pt"
        self.model_edit.setText(self.settings_store.value("model", str(default_model)))

    def _save_settings(self) -> None:
        self.settings_store.setValue("host", self.host_edit.text().strip())
        self.settings_store.setValue("command_port", self.command_port.value())
        self.settings_store.setValue("robot_host", self.robot_host_edit.text().strip())
        self.settings_store.setValue("token", self.token_edit.text())
        self.settings_store.setValue("camera_source", self.camera_source.currentData())
        self.settings_store.setValue("camera_index", self.camera_index.value())
        self.settings_store.setValue("camera_url", self.camera_url.text().strip())
        self.settings_store.setValue("model", self.model_edit.text().strip())
        self.settings_store.setValue("nav_x", self.nav_x.value())
        self.settings_store.setValue("nav_y", self.nav_y.value())
        self.settings_store.setValue("nav_yaw", self.nav_yaw.value())

    def _update_camera_fields(self) -> None:
        source = self.camera_source.currentData()
        local_camera = source == "local"
        self.camera_index.setEnabled(local_camera)
        self.camera_url.setEnabled(source == "network")

    def choose_model(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "YOLO 모델 선택", self.model_edit.text(), "PyTorch model (*.pt)"
        )
        if path:
            self.model_edit.setText(path)

    def toggle_connection(self) -> None:
        if self.client is not None:
            self.disconnect_robot()
            return
        host = self.host_edit.text().strip()
        if not host:
            QMessageBox.warning(self, "연결", "Ubuntu VM IP를 입력하세요.")
            return
        self._save_settings()
        self.client = RobotClient(
            host,
            self.command_port.value(),
            self.token_edit.text(),
            self.robot_host_edit.text(),
            self,
        )
        self.client.connection_changed.connect(self.on_connection_changed)
        self.client.status_received.connect(self.on_status)
        self.client.log_message.connect(self.append_log)
        self.client.start()
        self.connect_button.setText("연결 해제")
        self.host_edit.setEnabled(False)
        self.command_port.setEnabled(False)
        self.robot_host_edit.setEnabled(False)
        self.token_edit.setEnabled(False)

    def _auto_connect(self) -> None:
        if self.client is not None:
            return
        if not self.host_edit.text().strip():
            self.append_log("Ubuntu VM IP가 없어 자동 연결을 건너뜁니다")
            return
        self.append_log("저장된 Ubuntu VM으로 자동 연결합니다")
        self.toggle_connection()

    def disconnect_robot(self) -> None:
        self._manual_keys.clear()
        client = self.client
        if client is not None:
            # Latch emergency stop before waiting for camera/navigation shutdown.
            client.stop()
        self.stop_vision()
        self.stop_navigation()
        if client is not None:
            if client.wait(3000):
                client.deleteLater()
            else:
                self.append_log("통신 작업자가 종료 대기 중입니다")
        self.client = None
        self.gateway_connected = False
        self.robot_connected = False
        self.current_robot_ip = ""
        self._last_robot_state = None
        self.connect_button.setText("서버에 연결")
        self.host_edit.setEnabled(True)
        self.command_port.setEnabled(True)
        self.robot_host_edit.setEnabled(True)
        self.token_edit.setEnabled(True)
        self._set_connection_status("●  연결 안 됨", "offline")
        self.robot_address_label.setText("-")

    def on_connection_changed(self, connected: bool, message: str) -> None:
        self.gateway_connected = connected
        if not connected:
            self.robot_connected = False
            self.current_robot_ip = ""
        state = "online" if connected else "offline"
        self._set_connection_status(f"●  {message}", state)
        self.append_log(message)

    def on_status(self, status: dict) -> None:
        relay_status = "robot_connected" in status
        if relay_status:
            self.robot_connected = bool(status.get("robot_connected"))
            resolved_robot_ip = str(status.get("robot_ip") or "").strip()
            self.current_robot_ip = resolved_robot_ip if self.robot_connected else ""
            robot_ip = resolved_robot_ip or "탐색 중"
            method = str(status.get("discovery_method") or "-")
            self.robot_address_label.setText(f"{robot_ip} ({method})")
            if self.robot_connected:
                self._set_connection_status(
                    f"●  VM · 로봇 {robot_ip} 연결됨", "online"
                )
                self.lidar_label.setText("로봇 내부 안전제어")
            else:
                self._set_connection_status(
                    "●  VM 연결됨 · 로봇 재연결 중", "waiting"
                )
                self.lidar_label.setText("로봇 연결 안 됨")
            state = (self.robot_connected, robot_ip)
            if state != self._last_robot_state:
                error = status.get("robot_error")
                detail = f": {error}" if error else ""
                self.append_log(
                    f"로봇 {'연결됨' if self.robot_connected else '연결 대기'} "
                    f"{robot_ip}{detail}"
                )
                self._last_robot_state = state
        else:
            self.robot_connected = self.gateway_connected
            self.robot_address_label.setText("ROS 직접 연결")
            lidar_ok = bool(status.get("lidar_ok"))
            self.lidar_label.setText("정상" if lidar_ok else "데이터 없음/지연")
        distance = status.get("front_distance")
        self.distance_label.setText("-" if distance is None else f"{distance:.2f} m")
        self.avoid_label.setText(str(status.get("avoid_state", "-")))
        self.applied_label.setText(
            f"linear={float(status.get('applied_linear', 0.0)):.2f} · "
            f"angular={float(status.get('applied_angular', 0.0)):.2f}"
        )

    def _set_connection_status(self, text: str, state: str) -> None:
        self.connection_label.setText(text)
        self.connection_label.setProperty("state", state)
        style = self.connection_label.style()
        style.unpolish(self.connection_label)
        style.polish(self.connection_label)

    def toggle_follow(self, checked: bool) -> None:
        if checked:
            self.start_follow()
        else:
            self.stop_vision()

    def toggle_perception(self, checked: bool) -> None:
        if checked:
            self.start_perception()
        else:
            self.stop_vision()

    def start_perception(self) -> None:
        self._start_vision("perception")

    def start_follow(self) -> None:
        if self.client is None or not self.robot_connected:
            self.follow_button.setChecked(False)
            QMessageBox.warning(
                self, "Follow Me", "먼저 Ubuntu VM과 로봇에 연결하세요."
            )
            return
        self._start_vision("follow")

    def _start_vision(self, mode: str) -> None:
        if self.vision is not None:
            if self.vision_mode == mode:
                return
            self.stop_vision()
        model_path = self.model_edit.text().strip()
        if not Path(model_path).is_file():
            self.perception_button.setChecked(False)
            self.follow_button.setChecked(False)
            QMessageBox.warning(self, "Perception", "YOLO 모델 파일을 확인하세요.")
            return
        settings = FollowSettings(
            linear_speed=self.linear_speed.value(),
            angular_speed=self.angular_speed.value(),
            stop_height_ratio=self.stop_ratio.value(),
        )
        if self.camera_source.currentData() == "local":
            camera_input: int | str = self.camera_index.value()
        elif self.camera_source.currentData() == "robot":
            if not self.robot_connected or not self.current_robot_ip:
                self.perception_button.setChecked(False)
                self.follow_button.setChecked(False)
                QMessageBox.warning(
                    self,
                    "Perception",
                    "먼저 Ubuntu VM과 로봇에 연결하세요.",
                )
                return
            camera_input = f"http://{self.current_robot_ip}:8080/stream.mjpg"
            self.append_log(f"로봇 카메라 연결: {camera_input}")
        else:
            camera_input = self.camera_url.text().strip()
            if not camera_input:
                self.perception_button.setChecked(False)
                self.follow_button.setChecked(False)
                QMessageBox.warning(
                    self, "Perception", "네트워크 영상 URL을 입력하세요."
                )
                return
        self.stop_navigation()
        self.vision = VisionWorker(
            camera_input,
            model_path,
            settings,
            frame_skip=self.frame_skip.value(),
            parent=self,
        )
        self.vision.frame_ready.connect(self.show_frame)
        self.vision.command_ready.connect(self.on_follow_command)
        self.vision.metrics_ready.connect(self.metrics_label.setText)
        self.vision.error.connect(self.on_vision_error)
        self.vision.start()
        self.vision_mode = mode
        self.follow_active = mode == "follow"
        self.perception_button.setChecked(mode == "perception")
        self.perception_button.setText(
            "비전 인식 중지" if mode == "perception" else "비전 인식 시작"
        )
        self.follow_button.setChecked(mode == "follow")
        self.follow_button.setText(
            "Follow Me 중지" if mode == "follow" else "Follow Me"
        )
        if mode == "perception" and self.client is not None:
            self.client.set_command(0.0, 0.0)
        self.append_log(
            "Perception 시작 (영상/인식만)"
            if mode == "perception"
            else "Perception + Follow Me 통합 시작"
        )

    def stop_vision(self) -> None:
        self.follow_active = False
        self.vision_mode = None
        if self.client is not None:
            self.client.set_command(0.0, 0.0)
        if self.vision is not None:
            worker = self.vision
            self.vision = None
            worker.stop()
            if worker.wait(100):
                worker.deleteLater()
            else:
                worker.finished.connect(worker.deleteLater)
        self.perception_button.setChecked(False)
        self.perception_button.setText("비전 인식 시작")
        self.follow_button.setChecked(False)
        self.follow_button.setText("Follow Me")

    # Backwards-compatible name used by the connection shutdown path.
    def stop_follow(self) -> None:
        self.stop_vision()

    def toggle_navigation(self, checked: bool) -> None:
        if checked:
            self.start_navigation()
        else:
            self.stop_navigation()

    def start_navigation(self) -> None:
        configured = os.getenv("ROBOT_ROS2_EXECUTABLE", "ros2")
        executable = shutil.which(configured)
        if executable is None and Path(configured).is_file():
            executable = configured
        if executable is None:
            self.navigation_button.setChecked(False)
            QMessageBox.warning(
                self,
                "Navigation",
                "ros2 실행 파일을 찾을 수 없습니다.\n"
                "Navigation 버튼은 ROS2와 Nav2가 설치된 컴퓨터에서 "
                "GUI를 실행했을 때 사용할 수 있습니다.",
            )
            return

        self.stop_vision()
        self.stop_navigation()
        self._save_settings()
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(self._read_navigation_output)
        process.finished.connect(self._navigation_finished)
        process.errorOccurred.connect(
            lambda error: self.append_log(f"Navigation 프로세스 오류: {error}")
        )
        self.navigation_process = process
        arguments = [
            "run",
            "robot_project",
            "integrated_main",
            "--mode",
            "navigation",
            "--control-stdin",
            "--ros-args",
            "-p",
            f"goal_x:={self.nav_x.value()}",
            "-p",
            f"goal_y:={self.nav_y.value()}",
            "-p",
            f"goal_yaw:={self.nav_yaw.value()}",
        ]
        process.start(executable, arguments)
        if not process.waitForStarted(3000):
            self.navigation_process = None
            process.deleteLater()
            self.navigation_button.setChecked(False)
            QMessageBox.critical(
                self, "Navigation", "Navigation 통합 프로세스를 시작하지 못했습니다."
            )
            return
        self.navigation_button.setChecked(True)
        self.navigation_button.setText("Navigation 중지")
        self.append_log(
            "Navigation 시작: "
            f"x={self.nav_x.value():.2f}, y={self.nav_y.value():.2f}, "
            f"yaw={self.nav_yaw.value():.2f}"
        )

    def _read_navigation_output(self) -> None:
        if self.navigation_process is None:
            return
        output = bytes(self.navigation_process.readAllStandardOutput()).decode(
            errors="replace"
        )
        for line in output.splitlines():
            if line.strip():
                self.append_log(f"[Navigation] {line}")

    def _navigation_finished(self, exit_code: int, _status: object) -> None:
        process = self.navigation_process
        self.navigation_process = None
        self.navigation_button.setChecked(False)
        self.navigation_button.setText("Navigation")
        self.append_log(f"Navigation 종료 (code={exit_code})")
        if process is not None:
            process.deleteLater()

    def stop_navigation(self) -> None:
        process = self.navigation_process
        if process is not None:
            process.write(b"stop\n")
            process.waitForBytesWritten(300)
            if not process.waitForFinished(2000):
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()
                    process.waitForFinished(1000)
            if self.navigation_process is process:
                self.navigation_process = None
                process.deleteLater()
        self.navigation_button.setChecked(False)
        self.navigation_button.setText("Navigation")

    def stop_all_features(self) -> None:
        self.stop_vision()
        self.stop_navigation()
        self.stop_motion()
        self.append_log("전체 기능 중지")

    def on_follow_command(
        self, linear: float, angular: float, servo_pan: object, mode: str
    ) -> None:
        if self.follow_active and self.client is not None:
            pan = int(servo_pan) if servo_pan is not None else None
            self.client.set_command(linear, angular, pan)
            self.metrics_label.setText(
                f"{mode} · linear={linear:.2f} angular={angular:.2f} servo={pan}"
            )

    def manual_motion(self, linear_scale: float, angular_scale: float) -> None:
        if self.client is None or not self.robot_connected:
            return
        self.stop_vision()
        self.stop_navigation()
        linear = self.linear_speed.value() * linear_scale
        angular = self.angular_speed.value() * angular_scale
        self.client.set_command(linear, angular)

    def stop_motion(self) -> None:
        if self.client is not None:
            self.client.set_command(0.0, 0.0)

    def emergency_stop(self) -> None:
        self._manual_keys.clear()
        self.stop_vision()
        self.stop_navigation()
        if self.client is not None:
            self.client.emergency_stop()
        self.append_log("긴급 정지 명령")

    def show_frame(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        self.video.setPixmap(
            pixmap.scaled(
                self.video.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def on_vision_error(self, message: str) -> None:
        self.append_log(f"영상 오류: {message}")
        self.stop_follow()
        QMessageBox.critical(self, "영상/YOLO 오류", message)

    def append_log(self, message: str) -> None:
        self.log.appendPlainText(message)

    def closeEvent(self, event: QCloseEvent) -> None:
        self._save_settings()
        self.disconnect_robot()
        for worker in self.findChildren(VisionWorker):
            worker.stop()
            worker.wait(3000)
        event.accept()


def main() -> None:
    application = QApplication(sys.argv)
    application.setApplicationName("Robot Control v2")
    application.setStyle("Fusion")
    font = QFont()
    font.setPointSize(11)
    application.setFont(font)
    application.setStyleSheet(APP_STYLESHEET)
    window = MainWindow()
    window.show()
    raise SystemExit(application.exec())


if __name__ == "__main__":
    main()
