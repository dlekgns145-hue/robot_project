#!/usr/bin/env python3
"""Windows/macOS desktop control center for Robot Control v2."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QEvent, QSettings, QTimer, Qt
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
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from control_logic import FollowSettings
from map_navigation import NavigationMapView
from operations_dashboard import (
    OperationsDashboardPage,
    WorkCalendarPage,
    demo_snapshot,
)
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
        self._navigation_active = False
        self._map_requested_for_ip = ""
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
        settings_button = QPushButton("설정 열기")
        settings_button.setObjectName("HeaderSettingsButton")
        settings_button.setProperty("role", "quiet")
        settings_button.clicked.connect(self.open_settings_tab)
        header_layout.addWidget(settings_button)
        root_layout.addWidget(header)

        self.main_tabs = QTabWidget()
        self.main_tabs.setObjectName("MainTabs")
        root_layout.addWidget(self.main_tabs, 1)
        control_page = QWidget()
        control_page.setObjectName("ControlPage")
        self.main_tabs.addTab(control_page, "실시간 제어")
        body = QHBoxLayout()
        body.setContentsMargins(0, 4, 0, 0)
        body.setSpacing(18)
        control_page.setLayout(body)

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
        self.model_edit = QLineEdit()
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
        follow_form.addRow("전진 속도", self.linear_speed)
        follow_form.addRow("회전 속도", self.angular_speed)
        follow_form.addRow("정지 감도", self.stop_ratio)
        camera_hint = QLabel("카메라와 AI 모델은 ‘설정’ 탭에서 변경합니다.")
        camera_hint.setObjectName("ControlHint")
        camera_hint.setWordWrap(True)
        follow_form.addRow(camera_hint)
        navigation_hint = QLabel("목표 좌표는 ‘지도 주행’ 탭에서 선택합니다.")
        navigation_hint.setObjectName("ControlHint")
        navigation_hint.setWordWrap(True)
        follow_form.addRow(navigation_hint)
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
        right.insertWidget(0, manual_group)

        self.emergency_button = QPushButton("긴급 정지   EMERGENCY STOP")
        self.emergency_button.setObjectName("EmergencyButton")
        self.emergency_button.setMinimumHeight(58)
        self.emergency_button.clicked.connect(self.emergency_stop)
        right_column.addWidget(self.emergency_button)
        right.addStretch(1)

        self.operations_page = OperationsDashboardPage()
        self.operations_page.demo_requested.connect(self._show_operations_demo)
        self.calendar_page = WorkCalendarPage()
        self._build_navigation_page()
        self.main_tabs.addTab(self.operations_page, "운영 현황")
        self.main_tabs.addTab(self.calendar_page, "작업 캘린더")
        self._build_settings_page()

    def _build_settings_page(self) -> None:
        self.settings_page = QWidget()
        self.settings_page.setObjectName("SettingsPage")
        page_layout = QVBoxLayout(self.settings_page)
        page_layout.setContentsMargins(8, 10, 8, 12)
        page_layout.setSpacing(12)

        title = QLabel("설정")
        title.setObjectName("PageTitle")
        subtitle = QLabel(
            "서버 연결과 카메라 환경을 한곳에서 관리합니다. 변경값은 이 컴퓨터에 저장됩니다."
        )
        subtitle.setObjectName("Subtitle")
        page_layout.addWidget(title)
        page_layout.addWidget(subtitle)

        scroll = QScrollArea()
        scroll.setObjectName("SettingsScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        content = QWidget()
        content.setObjectName("SettingsContent")
        content_layout = QHBoxLayout(content)
        content_layout.setContentsMargins(0, 4, 4, 4)
        content_layout.setSpacing(16)

        network_group = QGroupBox("서버 및 로봇 연결")
        network_layout = QVBoxLayout(network_group)
        network_hint = QLabel(
            "GUI가 접속할 Ubuntu 서버와 서버가 제어할 로봇 주소를 입력하세요."
        )
        network_hint.setObjectName("SettingsHint")
        network_hint.setWordWrap(True)
        network_layout.addWidget(network_hint)
        connection_form = QFormLayout()
        connection_form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow
        )
        self.host_edit = QLineEdit()
        self.host_edit.setPlaceholderText("예: 172.30.1.81 또는 127.0.0.1")
        self.command_port = QSpinBox()
        self.command_port.setRange(1, 65535)
        self.robot_host_edit = QLineEdit()
        self.robot_host_edit.setPlaceholderText("예: raspberrypi.local")
        self.token_edit = QLineEdit()
        self.token_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.token_edit.setPlaceholderText("저장된 제어 토큰")
        connection_form.addRow("서버 IP / 호스트", self.host_edit)
        connection_form.addRow("명령 포트", self.command_port)
        connection_form.addRow("로봇 주소", self.robot_host_edit)
        connection_form.addRow("제어 토큰", self.token_edit)
        network_layout.addLayout(connection_form)
        self.settings_connection_label = QLabel("●  연결 안 됨")
        self.settings_connection_label.setObjectName("SettingsConnectionStatus")
        self.settings_connection_label.setProperty("state", "offline")
        network_layout.addWidget(self.settings_connection_label)
        network_actions = QHBoxLayout()
        save_button = QPushButton("설정 저장")
        save_button.setProperty("role", "quiet")
        save_button.clicked.connect(self.save_settings_from_ui)
        self.connect_button = QPushButton("서버에 연결")
        self.connect_button.setProperty("role", "primary")
        self.connect_button.clicked.connect(self.toggle_connection)
        for button in (save_button, self.connect_button):
            button.setMinimumHeight(44)
            network_actions.addWidget(button)
        network_layout.addLayout(network_actions)
        network_layout.addStretch(1)
        content_layout.addWidget(network_group, 1)

        camera_group = QGroupBox("카메라 및 AI")
        camera_layout = QVBoxLayout(camera_group)
        camera_hint = QLabel(
            "Follow Me와 비전 인식에 사용할 영상 입력과 YOLO 모델을 지정합니다."
        )
        camera_hint.setObjectName("SettingsHint")
        camera_hint.setWordWrap(True)
        camera_layout.addWidget(camera_hint)
        camera_form = QFormLayout()
        camera_form.setFieldGrowthPolicy(
            QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow
        )
        model_row = QHBoxLayout()
        browse_button = QPushButton("찾기")
        browse_button.setProperty("role", "quiet")
        browse_button.clicked.connect(self.choose_model)
        model_row.addWidget(self.model_edit, 1)
        model_row.addWidget(browse_button)
        camera_form.addRow("카메라 입력", self.camera_source)
        camera_form.addRow("카메라 번호", self.camera_index)
        camera_form.addRow("영상 URL", self.camera_url)
        camera_form.addRow("YOLO 모델", model_row)
        camera_form.addRow("추론 간격", self.frame_skip)
        camera_layout.addLayout(camera_form)
        camera_note = QLabel(
            "로봇 카메라(자동)는 현재 연결된 로봇의 영상 스트림을 사용합니다."
        )
        camera_note.setObjectName("SettingsInfo")
        camera_note.setWordWrap(True)
        camera_layout.addWidget(camera_note)
        camera_layout.addStretch(1)
        content_layout.addWidget(camera_group, 1)

        scroll.setWidget(content)
        page_layout.addWidget(scroll, 1)
        self.main_tabs.addTab(self.settings_page, "설정")

    def open_settings_tab(self) -> None:
        if hasattr(self, "settings_page"):
            self.main_tabs.setCurrentWidget(self.settings_page)

    def _build_navigation_page(self) -> None:
        page = QWidget()
        page.setObjectName("NavigationPage")
        layout = QHBoxLayout(page)
        layout.setContentsMargins(4, 10, 4, 4)
        layout.setSpacing(18)

        map_card = QFrame()
        map_card.setObjectName("PreviewCard")
        map_layout = QVBoxLayout(map_card)
        map_header = QHBoxLayout()
        title = QLabel("SAVED MAP NAVIGATION")
        title.setObjectName("SectionKicker")
        self.map_summary_label = QLabel("지도 대기 중")
        self.map_summary_label.setObjectName("VisionMetrics")
        map_header.addWidget(title)
        map_header.addStretch(1)
        map_header.addWidget(self.map_summary_label)
        map_layout.addLayout(map_header)
        self.map_view = NavigationMapView()
        self.map_view.goal_selected.connect(self._on_map_goal_selected)
        self.map_view.selection_rejected.connect(self.append_log)
        map_layout.addWidget(self.map_view, 1)
        legend = QLabel("파란색: 로봇 위치 · 주황색: 선택 목표 · 흰색: 주행 가능 영역")
        legend.setObjectName("ControlHint")
        legend.setAlignment(Qt.AlignmentFlag.AlignCenter)
        map_layout.addWidget(legend)
        layout.addWidget(map_card, 1)

        controls = QGroupBox("지도 목표 설정")
        controls.setMinimumWidth(310)
        controls.setMaximumWidth(380)
        controls_layout = QVBoxLayout(controls)
        explanation = QLabel(
            "지도에서 흰색 공간을 클릭한 뒤 방향(Yaw)을 정하고 이동 버튼을 누르세요."
        )
        explanation.setWordWrap(True)
        explanation.setObjectName("ControlHint")
        controls_layout.addWidget(explanation)
        form = QFormLayout()
        form.addRow("목표 X", self.nav_x)
        form.addRow("목표 Y", self.nav_y)
        form.addRow("목표 Yaw", self.nav_yaw)
        controls_layout.addLayout(form)
        self.map_goal_label = QLabel("지도에서 목표를 선택하세요")
        self.map_goal_label.setWordWrap(True)
        controls_layout.addWidget(self.map_goal_label)
        self.navigation_status_label = QLabel("Navigation 대기 중")
        self.navigation_status_label.setWordWrap(True)
        self.navigation_status_label.setObjectName("MetricValue")
        controls_layout.addWidget(self.navigation_status_label)
        controls_layout.addStretch(1)
        refresh_button = QPushButton("저장 지도 새로고침")
        refresh_button.clicked.connect(self.refresh_map)
        self.map_navigation_button = QPushButton("이 위치로 이동")
        self.map_navigation_button.setProperty("role", "primary")
        self.map_navigation_button.clicked.connect(self.start_navigation)
        cancel_button = QPushButton("Navigation 취소")
        cancel_button.setProperty("role", "quiet")
        cancel_button.clicked.connect(self.stop_navigation)
        for button in (refresh_button, self.map_navigation_button, cancel_button):
            button.setMinimumHeight(42)
            controls_layout.addWidget(button)
        layout.addWidget(controls)
        self.main_tabs.addTab(page, "지도 주행")

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
        self.linear_speed.setValue(
            float(self.settings_store.value("linear_speed", 0.35))
        )
        self.angular_speed.setValue(
            float(self.settings_store.value("angular_speed", 0.4))
        )
        self.stop_ratio.setValue(float(self.settings_store.value("stop_ratio", 0.55)))
        self.frame_skip.setValue(int(self.settings_store.value("frame_skip", 2)))
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
        self.settings_store.setValue("linear_speed", self.linear_speed.value())
        self.settings_store.setValue("angular_speed", self.angular_speed.value())
        self.settings_store.setValue("stop_ratio", self.stop_ratio.value())
        self.settings_store.setValue("frame_skip", self.frame_skip.value())
        self.settings_store.sync()

    def save_settings_from_ui(self) -> None:
        self._save_settings()
        self.append_log("연결 및 카메라 설정을 저장했습니다")

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
            QMessageBox.warning(self, "연결", "서버 IP 또는 호스트를 입력하세요.")
            return
        self._save_settings()
        self.client = RobotClient(
            host,
            self.command_port.value(),
            self.token_edit.text(),
            self.robot_host_edit.text(),
            self,
            admin_enabled=True,
        )
        self.client.connection_changed.connect(self.on_connection_changed)
        self.client.status_received.connect(self.on_status)
        self.client.map_received.connect(self.on_map_payload)
        self.client.operations_received.connect(self.on_operations_snapshot)
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
        self._map_requested_for_ip = ""
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
                if self.client is not None and self._map_requested_for_ip != robot_ip:
                    self._map_requested_for_ip = robot_ip
                    self.client.request_map()
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
        navigation = status.get("navigation")
        if isinstance(navigation, dict):
            self._update_navigation_status(navigation)

    def on_map_payload(self, payload: dict) -> None:
        try:
            self.map_view.set_map_payload(payload)
            width = int(payload["width"])
            height = int(payload["height"])
            resolution = float(payload["resolution"])
            self.map_summary_label.setText(
                f"{width * resolution:.2f} × {height * resolution:.2f} m"
            )
            self.append_log("로봇에서 저장 지도를 불러왔습니다")
        except (KeyError, TypeError, ValueError) as error:
            self._map_requested_for_ip = ""
            self.append_log(f"지도 표시 실패: {error}")

    def refresh_map(self) -> None:
        if self.client is None or not self.robot_connected:
            QMessageBox.warning(self, "지도", "먼저 Ubuntu VM과 로봇에 연결하세요.")
            return
        self.client.request_map()
        self.map_summary_label.setText("지도 불러오는 중")

    def _on_map_goal_selected(self, x: float, y: float) -> None:
        self.nav_x.setValue(x)
        self.nav_y.setValue(y)
        self.map_goal_label.setText(f"선택 목표: X={x:.2f}, Y={y:.2f}")
        self.append_log(f"지도 목표 선택: x={x:.2f}, y={y:.2f}")

    def _update_navigation_status(self, navigation: dict) -> None:
        state = str(navigation.get("state", "idle"))
        self._navigation_active = bool(navigation.get("active", False))
        message = str(navigation.get("message") or state)
        distance = navigation.get("distance_remaining")
        if distance is not None:
            message += f" · 남은 거리 {float(distance):.2f} m"
        self.navigation_status_label.setText(message)
        self.map_view.set_robot_pose(navigation.get("pose"))
        self.navigation_button.setChecked(self._navigation_active)
        self.navigation_button.setText(
            "Navigation 중지" if self._navigation_active else "Navigation"
        )
        self.map_navigation_button.setEnabled(not self._navigation_active)

    def on_operations_snapshot(self, snapshot: dict) -> None:
        self.operations_page.update_snapshot(snapshot)
        self.calendar_page.update_snapshot(snapshot)

    def _show_operations_demo(self) -> None:
        snapshot = demo_snapshot()
        self.operations_page.update_snapshot(snapshot, demo=True)
        self.calendar_page.update_snapshot(snapshot)
        self.append_log("관리자 운영 페이지 샘플 데이터를 표시합니다")

    def _set_connection_status(self, text: str, state: str) -> None:
        self.connection_label.setText(text)
        self.connection_label.setProperty("state", state)
        style = self.connection_label.style()
        style.unpolish(self.connection_label)
        style.polish(self.connection_label)
        if hasattr(self, "settings_connection_label"):
            self.settings_connection_label.setText(text)
            self.settings_connection_label.setProperty("state", state)
            settings_style = self.settings_connection_label.style()
            settings_style.unpolish(self.settings_connection_label)
            settings_style.polish(self.settings_connection_label)

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
        if self.client is None or not self.robot_connected:
            self.navigation_button.setChecked(False)
            QMessageBox.warning(
                self,
                "Navigation",
                "먼저 Ubuntu VM과 로봇에 연결하세요.",
            )
            return
        if self._navigation_active:
            return
        answer = QMessageBox.question(
            self,
            "Navigation 출발 확인",
            "로봇 주변과 목표까지의 통로에 사람·물건이 없습니까?\n"
            f"목표: X={self.nav_x.value():.2f}, Y={self.nav_y.value():.2f}, "
            f"Yaw={self.nav_yaw.value():.2f}",
        )
        if answer != QMessageBox.StandardButton.Yes:
            self.navigation_button.setChecked(False)
            return
        self.stop_vision()
        self._save_settings()
        self.client.navigate_to(
            self.nav_x.value(), self.nav_y.value(), self.nav_yaw.value()
        )
        self._navigation_active = True
        self.navigation_button.setChecked(True)
        self.navigation_button.setText("Navigation 중지")
        self.map_navigation_button.setEnabled(False)
        self.navigation_status_label.setText("Nav2 목표 전송 중")
        self.append_log(
            "Navigation 시작: "
            f"x={self.nav_x.value():.2f}, y={self.nav_y.value():.2f}, "
            f"yaw={self.nav_yaw.value():.2f}"
        )

    def stop_navigation(self) -> None:
        if self._navigation_active and self.client is not None:
            self.client.cancel_navigation()
            self.navigation_status_label.setText("Navigation 취소 요청 전송 중")
            self.append_log("Navigation 취소 요청")
        self._navigation_active = False
        self.navigation_button.setChecked(False)
        self.navigation_button.setText("Navigation")
        self.map_navigation_button.setEnabled(True)

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
