#!/usr/bin/env python3
"""Simplified desktop controller intended for everyday robot users."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QEvent, QSettings, QTimer, Qt
from PySide6.QtGui import QCloseEvent, QFont, QImage, QKeyEvent, QPixmap
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QComboBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from theme import USER_STYLESHEET


# Reuse the proven connection, safety, and vision workers from the administrator
# GUI. Keeping those workers shared prevents the two applications from drifting
# into different robot protocols.
DESKTOP_GUI_DIR = Path(__file__).resolve().parents[1] / "desktop_gui"
if str(DESKTOP_GUI_DIR) not in sys.path:
    sys.path.insert(0, str(DESKTOP_GUI_DIR))

from control_logic import FollowSettings  # noqa: E402
from robot_client import RobotClient  # noqa: E402
from vision_worker import VisionWorker  # noqa: E402


class UserMainWindow(QMainWindow):
    """Daily-use UI with advanced administrator controls intentionally omitted."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Companion")
        self.resize(1180, 780)
        self.setMinimumSize(900, 640)

        self.settings_store = QSettings("robot-project", "robot-user-control-v2")
        self.client: RobotClient | None = None
        self.vision: VisionWorker | None = None
        self.vision_mode: str | None = None
        self.gateway_connected = False
        self.robot_connected = False
        self.current_robot_ip = ""
        self._manual_keys: set[int] = set()
        self._last_robot_state: tuple[bool, str] | None = None

        self._build_ui()
        self._load_settings()
        self._set_controls_enabled(False)
        QTimer.singleShot(450, self._auto_connect)

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("AppRoot")
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(22, 20, 22, 22)
        root_layout.setSpacing(16)

        header = QFrame()
        header.setObjectName("Header")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(22, 16, 18, 16)

        brand = QVBoxLayout()
        brand.setSpacing(2)
        eyebrow = QLabel("ROBOT COMPANION")
        eyebrow.setObjectName("Eyebrow")
        title = QLabel("로봇과 함께하기")
        title.setObjectName("Title")
        subtitle = QLabel("카메라 확인 · Follow Me · 간편 운전")
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
        body.setSpacing(16)
        root_layout.addLayout(body, 1)

        left = QVBoxLayout()
        left.setSpacing(12)
        body.addLayout(left, 1)

        camera_card = QFrame()
        camera_card.setObjectName("CameraCard")
        camera_layout = QVBoxLayout(camera_card)
        camera_layout.setContentsMargins(16, 14, 16, 16)
        camera_layout.setSpacing(10)
        camera_header = QHBoxLayout()
        camera_title = QLabel("로봇 카메라")
        camera_title.setObjectName("StatusValue")
        self.camera_metrics = QLabel("카메라 꺼짐")
        self.camera_metrics.setObjectName("CameraMetrics")
        camera_header.addWidget(camera_title)
        camera_header.addStretch(1)
        camera_header.addWidget(self.camera_metrics)
        camera_layout.addLayout(camera_header)

        self.video = QLabel("로봇에 연결한 다음\n카메라 보기를 눌러주세요")
        self.video.setObjectName("VideoSurface")
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(460, 300)
        self.video.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        camera_layout.addWidget(self.video, 1)
        left.addWidget(camera_card, 1)

        actions = QHBoxLayout()
        actions.setSpacing(10)
        self.camera_button = QPushButton("카메라 보기")
        self.camera_button.setProperty("role", "primary")
        self.camera_button.setCheckable(True)
        self.camera_button.setMinimumHeight(48)
        self.camera_button.clicked.connect(self.toggle_camera)
        self.follow_button = QPushButton("Follow Me 시작")
        self.follow_button.setProperty("role", "primary")
        self.follow_button.setCheckable(True)
        self.follow_button.setMinimumHeight(48)
        self.follow_button.clicked.connect(self.toggle_follow)
        actions.addWidget(self.camera_button)
        actions.addWidget(self.follow_button)
        left.addLayout(actions)

        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.Shape.NoFrame)
        right_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        right_scroll.setMinimumWidth(340)
        right_scroll.setMaximumWidth(390)
        right_column = QVBoxLayout()
        right_column.setSpacing(10)
        right_column.addWidget(right_scroll, 1)
        body.addLayout(right_column)

        side = QWidget()
        side.setObjectName("SidePanel")
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 4, 0)
        side_layout.setSpacing(12)
        right_scroll.setWidget(side)

        status_card = QFrame()
        status_card.setObjectName("StatusCard")
        status_layout = QGridLayout(status_card)
        status_layout.setContentsMargins(16, 14, 16, 14)
        status_layout.setHorizontalSpacing(12)
        status_layout.setVerticalSpacing(4)
        address_caption = QLabel("현재 로봇")
        address_caption.setObjectName("StatusCaption")
        self.robot_address_label = QLabel("연결 대기 중")
        self.robot_address_label.setObjectName("StatusValue")
        mode_caption = QLabel("현재 모드")
        mode_caption.setObjectName("StatusCaption")
        self.mode_label = QLabel("대기")
        self.mode_label.setObjectName("StatusValue")
        status_layout.addWidget(address_caption, 0, 0)
        status_layout.addWidget(mode_caption, 0, 1)
        status_layout.addWidget(self.robot_address_label, 1, 0)
        status_layout.addWidget(self.mode_label, 1, 1)
        side_layout.addWidget(status_card)

        drive_group = QGroupBox("간편 운전")
        drive = QGridLayout(drive_group)
        drive.setSpacing(9)
        self.forward_button = QPushButton("▲\n전진")
        self.left_button = QPushButton("◀\n왼쪽")
        self.stop_button = QPushButton("■\n정지")
        self.right_button = QPushButton("▶\n오른쪽")
        self.backward_button = QPushButton("▼\n후진")
        for button in (
            self.forward_button,
            self.left_button,
            self.right_button,
            self.backward_button,
        ):
            button.setProperty("role", "drive")
        self.stop_button.setProperty("role", "stop")
        drive.addWidget(self.forward_button, 0, 1)
        drive.addWidget(self.left_button, 1, 0)
        drive.addWidget(self.stop_button, 1, 1)
        drive.addWidget(self.right_button, 1, 2)
        drive.addWidget(self.backward_button, 2, 1)
        self._bind_hold_button(self.forward_button, 1.0, 0.0)
        self._bind_hold_button(self.backward_button, -0.6, 0.0)
        self._bind_hold_button(self.left_button, 0.0, 1.0)
        self._bind_hold_button(self.right_button, 0.0, -1.0)
        self.stop_button.clicked.connect(self.stop_motion)

        speed_label = QLabel("운전 속도")
        speed_label.setObjectName("Hint")
        self.speed_combo = QComboBox()
        self.speed_combo.addItem("천천히", (0.18, 0.28))
        self.speed_combo.addItem("보통", (0.28, 0.38))
        self.speed_combo.addItem("빠르게", (0.38, 0.50))
        self.speed_combo.setCurrentIndex(1)
        drive.addWidget(speed_label, 3, 0)
        drive.addWidget(self.speed_combo, 3, 1, 1, 2)
        keyboard_hint = QLabel("키보드 W A S D / 방향키 · Space 정지")
        keyboard_hint.setObjectName("Hint")
        keyboard_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        drive.addWidget(keyboard_hint, 4, 0, 1, 3)
        side_layout.addWidget(drive_group)

        connection_group = QGroupBox("연결 설정 · 처음 한 번만")
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
        self.token_edit.setPlaceholderText("관리자에게 받은 토큰")
        self.connect_button = QPushButton("로봇 연결")
        self.connect_button.setProperty("role", "primary")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_form.addRow("Ubuntu VM IP", self.host_edit)
        connection_form.addRow("명령 포트", self.command_port)
        connection_form.addRow("로봇 주소", self.robot_host_edit)
        connection_form.addRow("제어 토큰", self.token_edit)
        connection_form.addRow(self.connect_button)
        side_layout.addWidget(connection_group)
        side_layout.addStretch(1)

        self.emergency_button = QPushButton("긴급 정지  EMERGENCY STOP")
        self.emergency_button.setObjectName("EmergencyButton")
        self.emergency_button.clicked.connect(self.emergency_stop)
        right_column.addWidget(self.emergency_button)

        self.drive_buttons = (
            self.forward_button,
            self.left_button,
            self.stop_button,
            self.right_button,
            self.backward_button,
        )

    def _bind_hold_button(
        self, button: QPushButton, linear_scale: float, angular_scale: float
    ) -> None:
        button.pressed.connect(lambda: self.manual_motion(linear_scale, angular_scale))
        button.released.connect(self.stop_motion)

    def _load_settings(self) -> None:
        self.host_edit.setText(str(self.settings_store.value("host", "")))
        self.command_port.setValue(
            int(self.settings_store.value("command_port", 9999))
        )
        self.robot_host_edit.setText(
            str(self.settings_store.value("robot_host", "raspberrypi.local"))
        )
        self.token_edit.setText(str(self.settings_store.value("token", "")))
        speed_index = int(self.settings_store.value("speed_index", 1))
        self.speed_combo.setCurrentIndex(max(0, min(speed_index, 2)))

    def _save_settings(self) -> None:
        self.settings_store.setValue("host", self.host_edit.text().strip())
        self.settings_store.setValue("command_port", self.command_port.value())
        self.settings_store.setValue(
            "robot_host", self.robot_host_edit.text().strip()
        )
        self.settings_store.setValue("token", self.token_edit.text())
        self.settings_store.setValue("speed_index", self.speed_combo.currentIndex())

    def _auto_connect(self) -> None:
        if self.client is None and self.host_edit.text().strip():
            self.toggle_connection()

    def toggle_connection(self) -> None:
        if self.client is not None:
            self.disconnect_robot()
            return

        host = self.host_edit.text().strip()
        if not host:
            QMessageBox.warning(self, "연결", "Ubuntu VM IP를 입력해주세요.")
            return

        self._save_settings()
        self.client = RobotClient(
            host,
            self.command_port.value(),
            self.token_edit.text(),
            self.robot_host_edit.text().strip(),
            self,
        )
        self.client.connection_changed.connect(self.on_connection_changed)
        self.client.status_received.connect(self.on_status)
        self.client.start()
        self.connect_button.setText("연결 해제")
        self._set_connection_fields_enabled(False)

    def disconnect_robot(self) -> None:
        self._manual_keys.clear()
        self.stop_vision()
        client = self.client
        if client is not None:
            client.stop()
            if client.wait(3000):
                client.deleteLater()
        self.client = None
        self.gateway_connected = False
        self.robot_connected = False
        self.current_robot_ip = ""
        self._last_robot_state = None
        self.connect_button.setText("로봇 연결")
        self._set_connection_fields_enabled(True)
        self._set_controls_enabled(False)
        self._set_connection_status("●  연결 안 됨", "offline")
        self.robot_address_label.setText("연결 대기 중")
        self.mode_label.setText("대기")

    def _set_connection_fields_enabled(self, enabled: bool) -> None:
        self.host_edit.setEnabled(enabled)
        self.command_port.setEnabled(enabled)
        self.robot_host_edit.setEnabled(enabled)
        self.token_edit.setEnabled(enabled)

    def _set_controls_enabled(self, enabled: bool) -> None:
        self.camera_button.setEnabled(enabled)
        self.follow_button.setEnabled(enabled)
        self.speed_combo.setEnabled(enabled)
        for button in self.drive_buttons:
            button.setEnabled(enabled)

    def on_connection_changed(self, connected: bool, message: str) -> None:
        self.gateway_connected = connected
        if not connected:
            self.robot_connected = False
            self.current_robot_ip = ""
            self._set_controls_enabled(False)
        self._set_connection_status(
            f"●  {message}", "waiting" if connected else "offline"
        )

    def on_status(self, status: dict) -> None:
        self.robot_connected = bool(status.get("robot_connected"))
        robot_ip = str(status.get("robot_ip") or "").strip()
        self.current_robot_ip = robot_ip if self.robot_connected else ""
        self.robot_address_label.setText(robot_ip or "로봇 찾는 중")
        self._set_controls_enabled(self.robot_connected)

        if self.robot_connected:
            self._set_connection_status("●  로봇 사용 가능", "online")
        else:
            self._set_connection_status("●  로봇 다시 연결하는 중", "waiting")
            if self.vision is not None:
                self.stop_vision()

        state = (self.robot_connected, robot_ip)
        if state != self._last_robot_state:
            self._last_robot_state = state

    def _set_connection_status(self, text: str, state: str) -> None:
        self.connection_label.setText(text)
        self.connection_label.setProperty("state", state)
        style = self.connection_label.style()
        style.unpolish(self.connection_label)
        style.polish(self.connection_label)

    def toggle_camera(self, checked: bool) -> None:
        if checked:
            self._start_vision("camera")
        elif self.vision_mode == "camera":
            self.stop_vision()

    def toggle_follow(self, checked: bool) -> None:
        if checked:
            self._start_vision("follow")
        elif self.vision_mode == "follow":
            self.stop_vision()

    def _start_vision(self, mode: str) -> None:
        if not self.robot_connected or not self.current_robot_ip:
            self.camera_button.setChecked(False)
            self.follow_button.setChecked(False)
            QMessageBox.warning(self, "카메라", "먼저 로봇 연결을 확인해주세요.")
            return

        model_path = Path(__file__).resolve().parents[2] / "yolov8n-pose.pt"
        if not model_path.is_file():
            self.camera_button.setChecked(False)
            self.follow_button.setChecked(False)
            QMessageBox.critical(
                self,
                "카메라",
                f"인식 모델 파일을 찾을 수 없습니다.\n{model_path}",
            )
            return

        self.stop_vision()
        linear_speed, angular_speed = self.speed_combo.currentData()
        settings = FollowSettings(
            linear_speed=float(linear_speed),
            angular_speed=float(angular_speed),
            stop_height_ratio=0.55,
        )
        camera_input = f"http://{self.current_robot_ip}:8080/stream.mjpg"
        self.vision = VisionWorker(
            camera_input,
            str(model_path),
            settings,
            frame_skip=2,
            parent=self,
        )
        self.vision.frame_ready.connect(self.show_frame)
        self.vision.command_ready.connect(self.on_vision_command)
        self.vision.metrics_ready.connect(self.camera_metrics.setText)
        self.vision.error.connect(self.on_vision_error)
        self.vision.start()
        self.vision_mode = mode
        self.camera_button.setChecked(mode == "camera")
        self.camera_button.setText(
            "카메라 끄기" if mode == "camera" else "카메라 보기"
        )
        self.follow_button.setChecked(mode == "follow")
        self.follow_button.setText(
            "Follow Me 중지" if mode == "follow" else "Follow Me 시작"
        )
        self.mode_label.setText("Follow Me" if mode == "follow" else "카메라")

    def stop_vision(self) -> None:
        if self.client is not None:
            self.client.set_command(0.0, 0.0)
        worker = self.vision
        self.vision = None
        self.vision_mode = None
        if worker is not None:
            worker.stop()
            if worker.wait(150):
                worker.deleteLater()
            else:
                worker.finished.connect(worker.deleteLater)
        self.camera_button.setChecked(False)
        self.camera_button.setText("카메라 보기")
        self.follow_button.setChecked(False)
        self.follow_button.setText("Follow Me 시작")
        self.camera_metrics.setText("카메라 꺼짐")
        self.video.clear()
        self.video.setText("로봇에 연결한 다음\n카메라 보기를 눌러주세요")
        self.mode_label.setText("대기")

    def on_vision_command(
        self, linear: float, angular: float, servo_pan: object, mode: str
    ) -> None:
        if self.vision_mode != "follow" or self.client is None:
            return
        pan = int(servo_pan) if servo_pan is not None else None
        self.client.set_command(linear, angular, pan)
        self.mode_label.setText(f"Follow · {mode}")

    def on_vision_error(self, message: str) -> None:
        self.stop_vision()
        QMessageBox.critical(self, "카메라 오류", message)

    def manual_motion(self, linear_scale: float, angular_scale: float) -> None:
        if self.client is None or not self.robot_connected:
            return
        if self.vision_mode == "follow":
            self.stop_vision()
        linear_speed, angular_speed = self.speed_combo.currentData()
        self.client.set_command(
            float(linear_speed) * linear_scale,
            float(angular_speed) * angular_scale,
        )
        self.mode_label.setText("수동 운전")

    def stop_motion(self) -> None:
        if self.client is not None:
            self.client.set_command(0.0, 0.0)
        if self.vision_mode is None:
            self.mode_label.setText("대기")

    def emergency_stop(self) -> None:
        self._manual_keys.clear()
        self.stop_vision()
        if self.client is not None:
            self.client.emergency_stop()
        self.mode_label.setText("긴급 정지")

    def show_frame(self, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        self.video.setPixmap(
            pixmap.scaled(
                self.video.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    @staticmethod
    def _direction_for_key(key: int) -> str | None:
        return {
            int(Qt.Key.Key_Up): "forward",
            int(Qt.Key.Key_W): "forward",
            int(Qt.Key.Key_Down): "backward",
            int(Qt.Key.Key_S): "backward",
            int(Qt.Key.Key_Left): "left",
            int(Qt.Key.Key_A): "left",
            int(Qt.Key.Key_Right): "right",
            int(Qt.Key.Key_D): "right",
        }.get(key)

    def _keyboard_control_blocked(self) -> bool:
        return isinstance(
            QApplication.focusWidget(),
            (QLineEdit, QAbstractSpinBox, QComboBox),
        )

    def _apply_keyboard_motion(self) -> None:
        directions = {
            direction
            for key in self._manual_keys
            if (direction := self._direction_for_key(key)) is not None
        }
        linear_scale = (
            1.0
            if "forward" in directions and "backward" not in directions
            else -0.6
            if "backward" in directions and "forward" not in directions
            else 0.0
        )
        angular_scale = (
            1.0
            if "left" in directions and "right" not in directions
            else -1.0
            if "right" in directions and "left" not in directions
            else 0.0
        )
        if linear_scale == 0.0 and angular_scale == 0.0:
            self.stop_motion()
        else:
            self.manual_motion(linear_scale, angular_scale)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        key = int(event.key())
        if key == int(Qt.Key.Key_Space) and not self._keyboard_control_blocked():
            self._manual_keys.clear()
            self.stop_motion()
            event.accept()
            return
        if self._direction_for_key(key) is None or self._keyboard_control_blocked():
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

    def closeEvent(self, event: QCloseEvent) -> None:
        self._save_settings()
        self.disconnect_robot()
        for worker in self.findChildren(VisionWorker):
            worker.stop()
            worker.wait(3000)
        event.accept()


def main() -> None:
    application = QApplication(sys.argv)
    application.setApplicationName("Robot Companion")
    application.setStyle("Fusion")
    font = QFont()
    font.setPointSize(11)
    application.setFont(font)
    application.setStyleSheet(USER_STYLESHEET)
    window = UserMainWindow()
    window.show()
    raise SystemExit(application.exec())


if __name__ == "__main__":
    main()
