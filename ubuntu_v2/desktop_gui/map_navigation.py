"""Saved occupancy-map display and map/world coordinate conversion."""

from __future__ import annotations

import base64
import math
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QImage, QMouseEvent, QPainter, QPaintEvent, QPen
from PySide6.QtWidgets import QLabel


@dataclass(frozen=True)
class MapMetadata:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    @classmethod
    def from_payload(cls, payload: dict) -> "MapMetadata":
        metadata = cls(
            width=int(payload["width"]),
            height=int(payload["height"]),
            resolution=float(payload["resolution"]),
            origin_x=float(payload["origin_x"]),
            origin_y=float(payload["origin_y"]),
            origin_yaw=float(payload.get("origin_yaw", 0.0)),
        )
        if metadata.width <= 0 or metadata.height <= 0 or metadata.resolution <= 0:
            raise ValueError("invalid map dimensions")
        return metadata


def pixel_to_world(pixel_x: float, pixel_y: float, map_info: MapMetadata) -> tuple[float, float]:
    local_x = (pixel_x + 0.5) * map_info.resolution
    local_y = (map_info.height - pixel_y - 0.5) * map_info.resolution
    cosine = math.cos(map_info.origin_yaw)
    sine = math.sin(map_info.origin_yaw)
    return (
        map_info.origin_x + cosine * local_x - sine * local_y,
        map_info.origin_y + sine * local_x + cosine * local_y,
    )


def world_to_pixel(world_x: float, world_y: float, map_info: MapMetadata) -> tuple[float, float]:
    delta_x = world_x - map_info.origin_x
    delta_y = world_y - map_info.origin_y
    cosine = math.cos(map_info.origin_yaw)
    sine = math.sin(map_info.origin_yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    return (
        local_x / map_info.resolution - 0.5,
        map_info.height - local_y / map_info.resolution - 0.5,
    )


class NavigationMapView(QLabel):
    goal_selected = Signal(float, float)
    selection_rejected = Signal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(500, 420)
        self.setText("저장 지도를 불러오는 중입니다")
        self.setStyleSheet("background:#eef3f7; border-radius:12px; color:#65788b;")
        self._image: QImage | None = None
        self._map_info: MapMetadata | None = None
        self._goal: tuple[float, float] | None = None
        self._robot_pose: tuple[float, float, float] | None = None

    @property
    def has_map(self) -> bool:
        return self._image is not None and self._map_info is not None

    def set_map_payload(self, payload: dict) -> None:
        map_info = MapMetadata.from_payload(payload)
        raw = base64.b64decode(str(payload["image_base64"]), validate=True)
        image = QImage.fromData(raw)
        if image.isNull():
            raise ValueError("PGM map image could not be decoded")
        if image.width() != map_info.width or image.height() != map_info.height:
            raise ValueError("map image and metadata dimensions differ")
        self._image = image.convertToFormat(QImage.Format.Format_Grayscale8)
        self._map_info = map_info
        self.setText("")
        self.update()

    def set_goal(self, x: float, y: float) -> None:
        self._goal = (float(x), float(y))
        self.update()

    def set_robot_pose(self, pose: dict | None) -> None:
        if not isinstance(pose, dict):
            self._robot_pose = None
        else:
            self._robot_pose = (
                float(pose["x"]),
                float(pose["y"]),
                float(pose.get("yaw", 0.0)),
            )
        self.update()

    def _image_rect(self) -> QRectF:
        if self._image is None:
            return QRectF()
        scale = min(self.width() / self._image.width(), self.height() / self._image.height())
        width = self._image.width() * scale
        height = self._image.height() * scale
        return QRectF((self.width() - width) / 2.0, (self.height() - height) / 2.0, width, height)

    def _world_view_point(self, x: float, y: float, rect: QRectF) -> QPointF | None:
        if self._image is None or self._map_info is None:
            return None
        pixel_x, pixel_y = world_to_pixel(x, y, self._map_info)
        if not (0 <= pixel_x < self._image.width() and 0 <= pixel_y < self._image.height()):
            return None
        return QPointF(
            rect.left() + pixel_x * rect.width() / self._image.width(),
            rect.top() + pixel_y * rect.height() / self._image.height(),
        )

    def paintEvent(self, event: QPaintEvent) -> None:
        if not self.has_map:
            super().paintEvent(event)
            return
        assert self._image is not None
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#eef3f7"))
        image_rect = self._image_rect()
        painter.drawImage(image_rect, self._image)
        painter.setPen(QPen(QColor("#bdcbd7"), 1))
        painter.drawRect(image_rect)

        if self._goal is not None:
            point = self._world_view_point(*self._goal, image_rect)
            if point is not None:
                painter.setPen(QPen(QColor("#ff9f43"), 3))
                painter.drawEllipse(point, 9, 9)
                painter.drawLine(point + QPointF(-13, 0), point + QPointF(13, 0))
                painter.drawLine(point + QPointF(0, -13), point + QPointF(0, 13))

        if self._robot_pose is not None:
            point = self._world_view_point(self._robot_pose[0], self._robot_pose[1], image_rect)
            if point is not None:
                yaw = self._robot_pose[2]
                length = 18.0
                tip = QPointF(point.x() + math.cos(yaw) * length, point.y() - math.sin(yaw) * length)
                painter.setBrush(QColor("#1976d2"))
                painter.setPen(QPen(QColor("#ffffff"), 2))
                painter.drawEllipse(point, 8, 8)
                painter.setPen(QPen(QColor("#1976d2"), 4))
                painter.drawLine(point, tip)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton or not self.has_map:
            super().mousePressEvent(event)
            return
        assert self._image is not None and self._map_info is not None
        rect = self._image_rect()
        position = event.position()
        if not rect.contains(position):
            self.selection_rejected.emit("지도 안쪽의 흰색 주행 가능 영역을 클릭하세요.")
            return
        pixel_x = (position.x() - rect.left()) * self._image.width() / rect.width()
        pixel_y = (position.y() - rect.top()) * self._image.height() / rect.height()
        image_x = min(self._image.width() - 1, max(0, int(pixel_x)))
        image_y = min(self._image.height() - 1, max(0, int(pixel_y)))
        if self._image.pixelColor(image_x, image_y).red() < 250:
            self.selection_rejected.emit("벽 또는 미확인 영역입니다. 흰색 공간을 선택하세요.")
            return
        world_x, world_y = pixel_to_world(pixel_x, pixel_y, self._map_info)
        self.set_goal(world_x, world_y)
        self.goal_selected.emit(world_x, world_y)
