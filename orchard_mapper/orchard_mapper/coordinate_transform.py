"""Coordinate conversions used by the visual mapper.

ROS convention:
  base_link +x points forward and +y points left.
Image convention:
  pixel +u points right and +v points down.
Global-map images store ROS +y upward, so image row zero is the north edge.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MapGeometry:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def validate(self) -> None:
        if self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must be positive")


@dataclass(frozen=True)
class BevGeometry:
    width: int
    height: int
    forward_range: float
    side_range: float

    def validate(self) -> None:
        if self.width < 2 or self.height < 2:
            raise ValueError("BEV dimensions must be at least 2x2")
        if self.forward_range <= 0.0 or self.side_range <= 0.0:
            raise ValueError("BEV metric ranges must be positive")


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def compose_pose(parent_to_child: Pose2D, child_to_object: Pose2D) -> Pose2D:
    """Return T_parent_object = T_parent_child * T_child_object."""

    cosine = math.cos(parent_to_child.yaw)
    sine = math.sin(parent_to_child.yaw)
    return Pose2D(
        x=parent_to_child.x + cosine * child_to_object.x - sine * child_to_object.y,
        y=parent_to_child.y + sine * child_to_object.x + cosine * child_to_object.y,
        yaw=normalize_angle(parent_to_child.yaw + child_to_object.yaw),
    )


def pose_delta(first: Pose2D, second: Pose2D) -> tuple[float, float]:
    return (
        math.hypot(second.x - first.x, second.y - first.y),
        abs(normalize_angle(second.yaw - first.yaw)),
    )


def bev_pixels_to_robot(pixels: np.ndarray, geometry: BevGeometry) -> np.ndarray:
    """Convert BEV pixels (u,v) to base_link coordinates (x,y).

    The bottom-centre BEV pixel is the robot origin. The top row is
    ``forward_range`` metres ahead. ``side_range`` means metres on each side,
    therefore the left/right image edges represent +side_range/-side_range.

      x_robot = (H-1-v)/(H-1) * forward_range
      y_robot = (0.5-u/(W-1)) * 2 * side_range
    """

    geometry.validate()
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    u = points[:, 0]
    v = points[:, 1]
    x_robot = (
        (geometry.height - 1.0 - v) / (geometry.height - 1.0) * geometry.forward_range
    )
    y_robot = (0.5 - u / (geometry.width - 1.0)) * 2.0 * geometry.side_range
    return np.column_stack((x_robot, y_robot))


def robot_to_map(points: np.ndarray, robot_pose: Pose2D) -> np.ndarray:
    """Apply the planar rigid transform T_map_base to base_link points."""

    local = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    cosine = math.cos(robot_pose.yaw)
    sine = math.sin(robot_pose.yaw)
    x_map = robot_pose.x + cosine * local[:, 0] - sine * local[:, 1]
    y_map = robot_pose.y + sine * local[:, 0] + cosine * local[:, 1]
    return np.column_stack((x_map, y_map))


def map_to_pixels(points: np.ndarray, geometry: MapGeometry) -> np.ndarray:
    """Convert ROS map metres to image pixels while preserving north-up.

    u = (x_map - origin_x) / resolution
    v = H - 1 - (y_map - origin_y) / resolution
    """

    geometry.validate()
    world = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    u = (world[:, 0] - geometry.origin_x) / geometry.resolution
    v = (
        geometry.height
        - 1.0
        - ((world[:, 1] - geometry.origin_y) / geometry.resolution)
    )
    return np.column_stack((u, v))


def bev_corners_to_map_pixels(
    bev: BevGeometry, robot_pose: Pose2D, map_geometry: MapGeometry
) -> np.ndarray:
    corners = np.asarray(
        [
            [0.0, 0.0],
            [bev.width - 1.0, 0.0],
            [bev.width - 1.0, bev.height - 1.0],
            [0.0, bev.height - 1.0],
        ],
        dtype=np.float64,
    )
    return map_to_pixels(
        robot_to_map(bev_pixels_to_robot(corners, bev), robot_pose),
        map_geometry,
    )


def pose_from_transform(transform) -> Pose2D:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return Pose2D(
        float(translation.x),
        float(translation.y),
        quaternion_to_yaw(
            float(rotation.x),
            float(rotation.y),
            float(rotation.z),
            float(rotation.w),
        ),
    )


def flattened_points(values: Iterable[float], *, count: int = 4) -> np.ndarray:
    numbers = [float(value) for value in values]
    if len(numbers) != count * 2:
        raise ValueError(f"expected {count * 2} point values, got {len(numbers)}")
    return np.asarray(numbers, dtype=np.float32).reshape(count, 2)
