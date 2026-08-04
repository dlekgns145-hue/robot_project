#!/usr/bin/env python3
"""Windows/macOS desktop control center for Robot Control v2."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QSettings, Qt
from PySide6.QtGui import QCloseEvent, QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
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
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from control_logic import FollowSettings
from robot_client import RobotClient
from vision_worker import VisionWorker


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Robot Control v2")
        self.resize(1060, 720)
        self.settings_store = QSettings("robot-project", "robot-control-v2")
        self.client: RobotClient | None = None
        self.vision: VisionWorker | None = None
        self.gateway_connected = False
        self.robot_connected = False
        self._last_robot_state: tuple[bool, str] | None = None
        self.follow_active = False
        self._build_ui()
        self._load_settings()

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)

        left = QVBoxLayout()
        right = QVBoxLayout()
        layout.addLayout(left, 3)
        layout.addLayout(right, 2)

        self.video = QLabel("Follow를 시작하면 선택한 카메라가 표시됩니다")
        self.video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video.setMinimumSize(640, 480)
        self.video.setStyleSheet("background:#15191f;color:#aab2bd;border-radius:8px;")
        left.addWidget(self.video, 1)

        self.metrics_label = QLabel("영상 대기 중")
        left.addWidget(self.metrics_label)

        connection_group = QGroupBox("연결")
        connection_form = QFormLayout(connection_group)
        self.host_edit = QLineEdit()
        self.command_port = QSpinBox()
        self.command_port.setRange(1, 65535)
        self.robot_host_edit = QLineEdit()
        self.robot_host_edit.setPlaceholderText("raspberrypi.local")
        self.token_edit = QLineEdit()
        self.token_edit.setEchoMode(QLineEdit.EchoMode.Password)
        self.connect_button = QPushButton("연결")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_label = QLabel("연결 안 됨")
        connection_form.addRow("Ubuntu VM IP", self.host_edit)
        connection_form.addRow("명령 포트", self.command_port)
        connection_form.addRow("로봇 이름(보조 탐색)", self.robot_host_edit)
        connection_form.addRow("제어 토큰", self.token_edit)
        connection_form.addRow(self.connect_button, self.connection_label)
        right.addWidget(connection_group)

        follow_group = QGroupBox("Follow")
        follow_form = QFormLayout(follow_group)
        self.camera_source = QComboBox()
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
        browse_button.clicked.connect(self.choose_model)
        model_row.addWidget(self.model_edit, 1)
        model_row.addWidget(browse_button)
        self.linear_speed = self._double_spin(0.0, 0.5, 0.35, 0.01)
        self.angular_speed = self._double_spin(0.0, 0.8, 0.4, 0.01)
        self.stop_ratio = self._double_spin(0.2, 0.9, 0.55, 0.01)
        self.frame_skip = QSpinBox()
        self.frame_skip.setRange(1, 10)
        self.frame_skip.setValue(2)
        self.follow_button = QPushButton("Follow 시작")
        self.follow_button.setCheckable(True)
        self.follow_button.clicked.connect(self.toggle_follow)
        follow_form.addRow("카메라 입력", self.camera_source)
        follow_form.addRow("카메라 번호", self.camera_index)
        follow_form.addRow("영상 URL", self.camera_url)
        follow_form.addRow("YOLO 모델", model_row)
        follow_form.addRow("전진 속도", self.linear_speed)
        follow_form.addRow("회전 속도", self.angular_speed)
        follow_form.addRow("정지 박스 비율", self.stop_ratio)
        follow_form.addRow("추론 프레임 간격", self.frame_skip)
        follow_form.addRow(self.follow_button)
        right.addWidget(follow_group)

        manual_group = QGroupBox("수동 조작")
        manual = QGridLayout(manual_group)
        forward = QPushButton("▲ 전진")
        left_turn = QPushButton("◀ 좌회전")
        stop = QPushButton("■ 정지")
        right_turn = QPushButton("우회전 ▶")
        backward = QPushButton("▼ 후진")
        manual.addWidget(forward, 0, 1)
        manual.addWidget(left_turn, 1, 0)
        manual.addWidget(stop, 1, 1)
        manual.addWidget(right_turn, 1, 2)
        manual.addWidget(backward, 2, 1)
        self._bind_hold_button(forward, 1.0, 0.0)
        self._bind_hold_button(backward, -0.6, 0.0)
        self._bind_hold_button(left_turn, 0.0, 1.0)
        self._bind_hold_button(right_turn, 0.0, -1.0)
        stop.clicked.connect(self.stop_motion)
        right.addWidget(manual_group)

        self.emergency_button = QPushButton("긴급 정지")
        self.emergency_button.setMinimumHeight(52)
        self.emergency_button.setStyleSheet(
            "QPushButton{background:#c62828;color:white;font-size:18px;font-weight:bold;}"
            "QPushButton:pressed{background:#8e0000;}"
        )
        self.emergency_button.clicked.connect(self.emergency_stop)
        right.addWidget(self.emergency_button)

        status_group = QGroupBox("로봇 상태")
        status_form = QFormLayout(status_group)
        self.robot_address_label = QLabel("-")
        self.lidar_label = QLabel("-")
        self.distance_label = QLabel("-")
        self.avoid_label = QLabel("-")
        self.applied_label = QLabel("-")
        status_form.addRow("현재 로봇 주소", self.robot_address_label)
        status_form.addRow("LiDAR", self.lidar_label)
        status_form.addRow("정면 거리", self.distance_label)
        status_form.addRow("회피 상태", self.avoid_label)
        status_form.addRow("실제 명령", self.applied_label)
        right.addWidget(status_group)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        right.addWidget(self.log, 1)

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

    def _load_settings(self) -> None:
        self.host_edit.setText(self.settings_store.value("host", ""))
        self.command_port.setValue(int(self.settings_store.value("command_port", 9999)))
        self.robot_host_edit.setText(
            self.settings_store.value("robot_host", "raspberrypi.local")
        )
        self.token_edit.setText(self.settings_store.value("token", ""))
        source = self.settings_store.value("camera_source", "local")
        source_index = self.camera_source.findData(source)
        self.camera_source.setCurrentIndex(max(source_index, 0))
        self.camera_index.setValue(int(self.settings_store.value("camera_index", 0)))
        self.camera_url.setText(self.settings_store.value("camera_url", ""))
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

    def _update_camera_fields(self) -> None:
        local_camera = self.camera_source.currentData() == "local"
        self.camera_index.setEnabled(local_camera)
        self.camera_url.setEnabled(not local_camera)

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

    def disconnect_robot(self) -> None:
        self.stop_follow()
        if self.client is not None:
            self.client.stop()
            if self.client.wait(3000):
                self.client.deleteLater()
            else:
                self.append_log("통신 작업자가 종료 대기 중입니다")
        self.client = None
        self.gateway_connected = False
        self.robot_connected = False
        self._last_robot_state = None
        self.connect_button.setText("연결")
        self.host_edit.setEnabled(True)
        self.command_port.setEnabled(True)
        self.robot_host_edit.setEnabled(True)
        self.token_edit.setEnabled(True)
        self.connection_label.setText("연결 안 됨")
        self.robot_address_label.setText("-")

    def on_connection_changed(self, connected: bool, message: str) -> None:
        self.gateway_connected = connected
        if not connected:
            self.robot_connected = False
        color = "#2e7d32" if connected else "#b71c1c"
        self.connection_label.setText(message)
        self.connection_label.setStyleSheet(f"color:{color};font-weight:bold;")
        self.append_log(message)

    def on_status(self, status: dict) -> None:
        relay_status = "robot_connected" in status
        if relay_status:
            self.robot_connected = bool(status.get("robot_connected"))
            robot_ip = str(status.get("robot_ip") or "탐색 중")
            method = str(status.get("discovery_method") or "-")
            self.robot_address_label.setText(f"{robot_ip} ({method})")
            if self.robot_connected:
                self.connection_label.setText(f"VM · 로봇 {robot_ip} 연결됨")
                self.connection_label.setStyleSheet(
                    "color:#2e7d32;font-weight:bold;"
                )
                self.lidar_label.setText("로봇 내부 안전제어")
            else:
                self.connection_label.setText("VM 연결됨 · 로봇 탐색/재연결 중")
                self.connection_label.setStyleSheet(
                    "color:#ef6c00;font-weight:bold;"
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

    def toggle_follow(self, checked: bool) -> None:
        if checked:
            self.start_follow()
        else:
            self.stop_follow()

    def start_follow(self) -> None:
        if self.client is None or not self.robot_connected:
            self.follow_button.setChecked(False)
            QMessageBox.warning(self, "Follow", "먼저 Ubuntu VM에 연결하세요.")
            return
        model_path = self.model_edit.text().strip()
        if not Path(model_path).is_file():
            self.follow_button.setChecked(False)
            QMessageBox.warning(self, "Follow", "YOLO 모델 파일을 확인하세요.")
            return
        if self.vision is not None:
            self.follow_button.setChecked(False)
            return
        settings = FollowSettings(
            linear_speed=self.linear_speed.value(),
            angular_speed=self.angular_speed.value(),
            stop_height_ratio=self.stop_ratio.value(),
        )
        if self.camera_source.currentData() == "local":
            camera_input: int | str = self.camera_index.value()
        else:
            camera_input = self.camera_url.text().strip()
            if not camera_input:
                self.follow_button.setChecked(False)
                QMessageBox.warning(self, "Follow", "네트워크 영상 URL을 입력하세요.")
                return
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
        self.follow_active = True
        self.follow_button.setChecked(True)
        self.follow_button.setText("Follow 중지")
        self.append_log("Follow 시작")

    def stop_follow(self) -> None:
        self.follow_active = False
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
        self.follow_button.setChecked(False)
        self.follow_button.setText("Follow 시작")

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
        self.stop_follow()
        linear = self.linear_speed.value() * linear_scale
        angular = self.angular_speed.value() * angular_scale
        self.client.set_command(linear, angular)

    def stop_motion(self) -> None:
        if self.client is not None:
            self.client.set_command(0.0, 0.0)

    def emergency_stop(self) -> None:
        self.stop_follow()
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
        for worker in self.findChildren(VisionWorker):
            worker.stop()
            worker.wait(3000)
        self.disconnect_robot()
        event.accept()


def main() -> None:
    application = QApplication(sys.argv)
    application.setApplicationName("Robot Control v2")
    window = MainWindow()
    window.show()
    raise SystemExit(application.exec())


if __name__ == "__main__":
    main()
