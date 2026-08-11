"""Fuse LiDAR endpoints with matching camera pixels for visual map layers.

The occupancy grid remains the navigation source of truth.  This module only
adds camera appearance to cells that the LiDAR/SLAM map already marks occupied.
It has no ROS dependency so calibration and geometry can be unit tested.
"""

from __future__ import annotations

import math
from collections import OrderedDict
from dataclasses import dataclass
from typing import Iterable, Sequence

import cv2
import numpy as np

from map_texture_core import SavedMapInfo, world_to_map_pixel


@dataclass(frozen=True)
class CameraLidarCalibration:
    horizontal_fov_deg: float = 68.0
    vertical_fov_deg: float = 50.0
    camera_yaw_offset_deg: float = 0.0
    camera_pitch_down_deg: float = 18.0
    camera_height_m: float = 0.24
    lidar_x_offset_m: float = -0.0046
    lidar_y_offset_m: float = 0.0
    sample_height_fraction: float = 0.16
    sample_half_width_pixels: int = 3
    scan_stride: int = 2

    def validate(self) -> None:
        finite = (
            self.horizontal_fov_deg,
            self.vertical_fov_deg,
            self.camera_yaw_offset_deg,
            self.camera_pitch_down_deg,
            self.camera_height_m,
            self.lidar_x_offset_m,
            self.lidar_y_offset_m,
            self.sample_height_fraction,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("camera/LiDAR calibration values must be finite")
        if not 5.0 <= self.horizontal_fov_deg < 179.0:
            raise ValueError("horizontal camera FOV must be between 5 and 179 degrees")
        if not 5.0 <= self.vertical_fov_deg < 179.0:
            raise ValueError("vertical camera FOV must be between 5 and 179 degrees")
        if self.camera_height_m <= 0.0:
            raise ValueError("camera height must be positive")
        if not 0.02 <= self.sample_height_fraction <= 0.8:
            raise ValueError("camera sample height fraction is outside allowed bounds")
        if self.sample_half_width_pixels < 0:
            raise ValueError("camera sample width cannot be negative")
        if self.scan_stride < 1:
            raise ValueError("LiDAR scan stride must be positive")


@dataclass(frozen=True)
class ObstacleObservation:
    x: float
    y: float
    bgr: tuple[int, int, int]
    weight: float
    hits: int = 1


class ObstacleTextureAccumulator:
    """Bounded world-coordinate color accumulator robust to SLAM map resizing."""

    def __init__(self, *, cell_size_m: float = 0.04, maximum_cells: int = 50000):
        if not math.isfinite(cell_size_m) or cell_size_m <= 0.0:
            raise ValueError("accumulator cell size must be positive")
        if maximum_cells < 1:
            raise ValueError("maximum accumulator cells must be positive")
        self.cell_size_m = float(cell_size_m)
        self.maximum_cells = int(maximum_cells)
        self._cells: OrderedDict[tuple[int, int], list] = OrderedDict()

    def clear(self) -> None:
        self._cells.clear()

    def __len__(self) -> int:
        return len(self._cells)

    def add(self, observations: Iterable[ObstacleObservation]) -> int:
        accepted = 0
        for observation in observations:
            values = (observation.x, observation.y, observation.weight)
            if not all(math.isfinite(value) for value in values):
                continue
            if observation.weight <= 0.0:
                continue
            key = (
                int(round(observation.x / self.cell_size_m)),
                int(round(observation.y / self.cell_size_m)),
            )
            color = np.asarray(observation.bgr, dtype=np.float64)
            if color.shape != (3,) or np.any(color < 0) or np.any(color > 255):
                continue
            weight = float(observation.weight)
            if key in self._cells:
                cell = self._cells.pop(key)
                cell[0] += observation.x * weight
                cell[1] += observation.y * weight
                cell[2] += color * weight
                cell[3] += weight
                cell[4] += max(1, int(observation.hits))
            else:
                if len(self._cells) >= self.maximum_cells:
                    self._cells.popitem(last=False)
                cell = [
                    observation.x * weight,
                    observation.y * weight,
                    color * weight,
                    weight,
                    max(1, int(observation.hits)),
                ]
            self._cells[key] = cell
            accepted += 1
        return accepted

    def observations(self) -> list[ObstacleObservation]:
        result = []
        for x_sum, y_sum, color_sum, weight, hits in self._cells.values():
            if weight <= 0.0:
                continue
            color = np.clip(np.rint(color_sum / weight), 0, 255).astype(np.uint8)
            result.append(
                ObstacleObservation(
                    x=float(x_sum / weight),
                    y=float(y_sum / weight),
                    bgr=(int(color[0]), int(color[1]), int(color[2])),
                    weight=float(weight),
                    hits=int(hits),
                )
            )
        return result


def _camera_patch_color(
    frame: np.ndarray,
    *,
    bearing: float,
    obstacle_range: float,
    calibration: CameraLidarCalibration,
) -> tuple[tuple[int, int, int], float] | None:
    height, width = frame.shape[:2]
    horizontal_fov = math.radians(calibration.horizontal_fov_deg)
    vertical_fov = math.radians(calibration.vertical_fov_deg)
    camera_bearing = bearing - math.radians(calibration.camera_yaw_offset_deg)
    camera_bearing = math.atan2(math.sin(camera_bearing), math.cos(camera_bearing))
    if abs(camera_bearing) > horizontal_fov / 2.0:
        return None

    focal_x = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    pixel_x = width / 2.0 - focal_x * math.tan(camera_bearing)
    half_width = int(calibration.sample_half_width_pixels)
    left = max(0, int(round(pixel_x)) - half_width)
    right = min(width, int(round(pixel_x)) + half_width + 1)
    if right <= left:
        return None

    focal_y = (height / 2.0) / math.tan(vertical_fov / 2.0)
    ground_angle = math.atan2(calibration.camera_height_m, obstacle_range)
    optical_delta = ground_angle - math.radians(calibration.camera_pitch_down_deg)
    contact_y = height / 2.0 + focal_y * math.tan(optical_delta)
    band_height = max(3, int(round(height * calibration.sample_height_fraction)))
    bottom = min(height, int(round(contact_y)) - 2)
    top = max(int(round(height * 0.08)), bottom - band_height)
    if bottom <= top or bottom < 0 or top >= height:
        return None

    patch = frame[top:bottom, left:right]
    if patch.size == 0:
        return None
    pixels = patch.reshape(-1, 3).astype(np.float32)
    luminance = pixels.mean(axis=1)
    useful = pixels[(luminance >= 8.0) & (luminance <= 248.0)]
    if len(useful) < max(3, len(pixels) // 5):
        return None
    color = np.median(useful, axis=0)
    dispersion = float(np.median(np.linalg.norm(useful - color, axis=1)))
    range_weight = 1.0 / (1.0 + 0.18 * obstacle_range)
    texture_weight = 1.0 / (1.0 + dispersion / 55.0)
    weight = max(0.08, range_weight * texture_weight)
    rounded = np.clip(np.rint(color), 0, 255).astype(np.uint8)
    return (int(rounded[0]), int(rounded[1]), int(rounded[2])), weight


def observations_from_scan(
    frame: np.ndarray,
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    calibration: CameraLidarCalibration,
) -> list[ObstacleObservation]:
    """Match visible LiDAR endpoints to camera pixels in the same robot pose."""
    calibration.validate()
    if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
        raise ValueError("camera frame must be a non-empty BGR image")
    pose_values = (robot_x, robot_y, robot_yaw, angle_min, angle_increment)
    if not all(math.isfinite(value) for value in pose_values):
        raise ValueError("scan and robot pose values must be finite")
    if angle_increment == 0.0:
        raise ValueError("LiDAR angle increment cannot be zero")

    observations = []
    pose_cosine = math.cos(robot_yaw)
    pose_sine = math.sin(robot_yaw)
    lidar_x = (
        robot_x
        + pose_cosine * calibration.lidar_x_offset_m
        - pose_sine * calibration.lidar_y_offset_m
    )
    lidar_y = (
        robot_y
        + pose_sine * calibration.lidar_x_offset_m
        + pose_cosine * calibration.lidar_y_offset_m
    )
    for index in range(0, len(ranges), calibration.scan_stride):
        obstacle_range = float(ranges[index])
        if not math.isfinite(obstacle_range):
            continue
        if obstacle_range < max(0.05, range_min) or obstacle_range > range_max:
            continue
        bearing = angle_min + index * angle_increment
        sampled = _camera_patch_color(
            frame,
            bearing=bearing,
            obstacle_range=obstacle_range,
            calibration=calibration,
        )
        if sampled is None:
            continue
        color, weight = sampled
        world_bearing = robot_yaw + bearing
        observations.append(
            ObstacleObservation(
                x=lidar_x + math.cos(world_bearing) * obstacle_range,
                y=lidar_y + math.sin(world_bearing) * obstacle_range,
                bgr=color,
                weight=weight,
            )
        )
    return observations


def occupancy_grid_to_pgm(data: Sequence[int], width: int, height: int) -> np.ndarray:
    """Convert ROS OccupancyGrid order/values to map_server trinary PGM pixels."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("occupancy grid dimensions do not match its data")
    occupancy = np.asarray(data, dtype=np.int16).reshape(height, width)
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[(occupancy >= 0) & (occupancy <= 25)] = 254
    pgm[occupancy >= 65] = 0
    return np.flipud(pgm)


def render_obstacle_texture(
    occupancy: np.ndarray,
    info: SavedMapInfo,
    observations: Sequence[ObstacleObservation],
    *,
    endpoint_tolerance_pixels: int = 2,
    paint_radius_pixels: int = 1,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Render camera colors only where the occupancy map confirms obstacles."""
    if occupancy.ndim != 2 or occupancy.shape != (info.height, info.width):
        raise ValueError("occupancy image dimensions do not match saved map metadata")
    if endpoint_tolerance_pixels < 0 or paint_radius_pixels < 0:
        raise ValueError("render radii cannot be negative")
    occupied = occupancy < 65
    color_sum = np.zeros((info.height, info.width, 3), dtype=np.float32)
    weight_sum = np.zeros((info.height, info.width), dtype=np.float32)
    hit_sum = np.zeros((info.height, info.width), dtype=np.int32)

    for observation in observations:
        pixel_x, pixel_y = world_to_map_pixel(observation.x, observation.y, info)
        center_x, center_y = int(round(pixel_x)), int(round(pixel_y))
        left = max(0, center_x - endpoint_tolerance_pixels)
        right = min(info.width, center_x + endpoint_tolerance_pixels + 1)
        top = max(0, center_y - endpoint_tolerance_pixels)
        bottom = min(info.height, center_y + endpoint_tolerance_pixels + 1)
        if left >= right or top >= bottom:
            continue
        local = occupied[top:bottom, left:right]
        candidates = np.argwhere(local)
        if candidates.size == 0:
            continue
        candidate_y, candidate_x = min(
            candidates,
            key=lambda point: (top + int(point[0]) - pixel_y) ** 2
            + (left + int(point[1]) - pixel_x) ** 2,
        )
        target_x, target_y = left + int(candidate_x), top + int(candidate_y)
        for y in range(max(0, target_y - paint_radius_pixels), min(info.height, target_y + paint_radius_pixels + 1)):
            for x in range(max(0, target_x - paint_radius_pixels), min(info.width, target_x + paint_radius_pixels + 1)):
                if not occupied[y, x]:
                    continue
                if (x - target_x) ** 2 + (y - target_y) ** 2 > paint_radius_pixels ** 2:
                    continue
                weight = max(0.01, float(observation.weight))
                color_sum[y, x] += np.asarray(observation.bgr, np.float32) * weight
                weight_sum[y, x] += weight
                hit_sum[y, x] += max(1, int(observation.hits))

    observed = weight_sum > 0.0
    averaged = color_sum / np.maximum(weight_sum[..., None], 0.001)
    camera_bgr = np.clip(np.rint(averaged), 0, 255).astype(np.uint8)
    layer = np.zeros((info.height, info.width, 4), dtype=np.uint8)
    layer[observed, :3] = camera_bgr[observed]
    confidence_alpha = 105.0 + 42.0 * np.log1p(hit_sum.astype(np.float32))
    layer[..., 3] = np.where(observed, np.clip(confidence_alpha, 0, 235), 0).astype(np.uint8)

    base = cv2.cvtColor(occupancy, cv2.COLOR_GRAY2BGR)
    return base, layer, {
        "observed_pixels": int(np.count_nonzero(observed)),
        "observation_cells": len(observations),
        "total_hits": int(hit_sum.sum()),
        "camera_bgr": camera_bgr,
        "observed_mask": observed,
    }
