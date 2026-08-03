"""YOLO pose inference worker that always keeps only the newest camera frame."""

from __future__ import annotations

import threading
import time
from typing import Any

import cv2
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QImage
from ultralytics import YOLO

from control_logic import FollowSettings, movement, servo_target, smooth_servo


PERSON_CLASS_ID = 0
UPPER_BODY_KEYPOINTS = (0, 1, 2, 3, 4, 5, 6)
KEYPOINT_CONFIDENCE = 0.5


def pick_upper_body_target(result: Any) -> tuple[float, float, float, float] | None:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    classes = boxes.cls.detach().cpu().numpy().astype(int)
    coordinates = boxes.xyxy.detach().cpu().numpy()
    person_indices = [
        index for index, class_id in enumerate(classes) if class_id == PERSON_CLASS_ID
    ]
    if not person_indices:
        return None

    best_index = max(
        person_indices,
        key=lambda index: (
            (coordinates[index][2] - coordinates[index][0])
            * (coordinates[index][3] - coordinates[index][1])
        ),
    )
    full_x1, full_y1, full_x2, full_y2 = coordinates[best_index]
    if result.keypoints is None or result.keypoints.conf is None:
        return (
            float((full_x1 + full_x2) / 2),
            float((full_y1 + full_y2) / 2),
            float(full_x2 - full_x1),
            float(full_y2 - full_y1),
        )

    points = result.keypoints.xy[best_index].detach().cpu().numpy()
    confidence = result.keypoints.conf[best_index].detach().cpu().numpy()
    valid = [
        points[index]
        for index in UPPER_BODY_KEYPOINTS
        if confidence[index] >= KEYPOINT_CONFIDENCE
    ]
    if len(valid) < 2:
        return (
            float((full_x1 + full_x2) / 2),
            float((full_y1 + full_y2) / 2),
            float(full_x2 - full_x1),
            float(full_y2 - full_y1),
        )

    xs = [point[0] for point in valid]
    ys = [point[1] for point in valid]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)
    pad_x = (x2 - x1) * 0.3 + 10
    pad_y = (y2 - y1) * 0.3 + 10
    x1, x2 = x1 - pad_x, x2 + pad_x
    y1, y2 = y1 - pad_y, y2 + pad_y
    return float((x1 + x2) / 2), float((y1 + y2) / 2), float(x2 - x1), float(y2 - y1)


class VisionWorker(QThread):
    frame_ready = Signal(QImage)
    command_ready = Signal(float, float, object, str)
    metrics_ready = Signal(str)
    error = Signal(str)

    def __init__(
        self,
        camera_input: int | str,
        model_path: str,
        settings: FollowSettings,
        frame_skip: int = 2,
        image_size: int = 320,
        parent: object | None = None,
    ) -> None:
        super().__init__(parent)
        self.camera_input = camera_input
        self.model_path = model_path
        self.settings = settings
        self.frame_skip = max(frame_skip, 1)
        self.image_size = image_size
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            model = YOLO(self.model_path)
            if isinstance(self.camera_input, int):
                capture = cv2.VideoCapture(self.camera_input)
            else:
                capture = cv2.VideoCapture(
                    self.camera_input,
                    cv2.CAP_FFMPEG,
                    [
                        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                        3000,
                        cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                        1500,
                    ],
                )
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                raise RuntimeError(f"카메라를 열 수 없습니다: {self.camera_input}")

            frame_number = 0
            lost_frames = 0
            searching = False
            servo_current = 0
            servo_goal: int | None = None
            last_box: tuple[float, float, float, float] | None = None
            fps_started_at = time.monotonic()
            displayed_frames = 0

            try:
                while not self._stop_event.is_set():
                    ok, frame = capture.read()
                    if not ok:
                        raise RuntimeError("카메라 프레임 수신이 중단됐습니다")
                    height, width = frame.shape[:2]
                    frame_number += 1
                    infer = frame_number % self.frame_skip == 0

                    if infer:
                        result = model(
                            frame,
                            verbose=False,
                            conf=0.5,
                            imgsz=self.image_size,
                        )[0]
                        detection = pick_upper_body_target(result)
                        if detection is not None:
                            lost_frames = 0
                            searching = False
                            last_box = detection
                            servo_goal = servo_target(
                                detection[0], width, self.settings
                            )
                        else:
                            lost_frames += 1
                            if lost_frames > self.settings.grace_frames:
                                last_box = None
                                servo_goal = None
                            if lost_frames >= self.settings.lost_frame_limit:
                                searching = True

                    if searching:
                        servo_current = 0
                        linear = 0.0
                        angular = (
                            self.settings.search_angular_speed
                            * self.settings.search_direction
                        )
                        servo_to_send: int | None = 0
                        mode = "SEARCH"
                    elif last_box is not None:
                        servo_current = smooth_servo(
                            servo_current, servo_goal, self.settings.servo_max_step
                        )
                        linear, angular, mode = movement(
                            servo_current, last_box[3], height, self.settings
                        )
                        servo_to_send = servo_current
                    else:
                        linear, angular, servo_to_send, mode = (
                            0.0,
                            0.0,
                            None,
                            "NO PERSON",
                        )

                    self.command_ready.emit(linear, angular, servo_to_send, mode)
                    if last_box is not None:
                        cx, cy, box_width, box_height = last_box
                        cv2.rectangle(
                            frame,
                            (int(cx - box_width / 2), int(cy - box_height / 2)),
                            (int(cx + box_width / 2), int(cy + box_height / 2)),
                            (0, 255, 0),
                            2,
                        )
                    cv2.putText(
                        frame,
                        mode,
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (0, 255, 0),
                        2,
                    )
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = QImage(
                        rgb.data,
                        width,
                        height,
                        rgb.strides[0],
                        QImage.Format.Format_RGB888,
                    ).copy()
                    self.frame_ready.emit(image)

                    displayed_frames += 1
                    elapsed = time.monotonic() - fps_started_at
                    if elapsed >= 1.0:
                        self.metrics_ready.emit(
                            f"영상 {displayed_frames / elapsed:.1f} FPS · 모드 {mode}"
                        )
                        fps_started_at = time.monotonic()
                        displayed_frames = 0
            finally:
                capture.release()
        except Exception as error:  # GUI worker must report model/camera failures.
            if not self._stop_event.is_set():
                self.error.emit(str(error))
