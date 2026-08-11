"""Metric homography placement and weighted/feather image blending."""

from __future__ import annotations

import threading

import cv2
import numpy as np

from .coordinate_transform import (
    BevGeometry,
    MapGeometry,
    Pose2D,
    bev_corners_to_map_pixels,
)


class WeightedCanvas:
    def __init__(self, geometry: MapGeometry) -> None:
        geometry.validate()
        self.geometry = geometry
        self._sum = np.zeros((geometry.height, geometry.width, 3), np.float32)
        self._weight = np.zeros((geometry.height, geometry.width), np.float32)
        self.lock = threading.RLock()

    def clear(self) -> None:
        with self.lock:
            self._sum.fill(0.0)
            self._weight.fill(0.0)

    def reconfigure(self, geometry: MapGeometry) -> None:
        """Resize/rebase the canvas while preserving overlapping world pixels."""

        geometry.validate()
        with self.lock:
            old = self.geometry
            if geometry == old:
                return
            if abs(geometry.resolution - old.resolution) > 1e-9:
                self.geometry = geometry
                self._sum = np.zeros((geometry.height, geometry.width, 3), np.float32)
                self._weight = np.zeros((geometry.height, geometry.width), np.float32)
                return

            new_sum = np.zeros((geometry.height, geometry.width, 3), np.float32)
            new_weight = np.zeros((geometry.height, geometry.width), np.float32)
            u_offset = round((old.origin_x - geometry.origin_x) / old.resolution)
            y_offset = round((old.origin_y - geometry.origin_y) / old.resolution)
            v_offset = geometry.height - old.height - y_offset

            old_u0 = max(0, -u_offset)
            old_v0 = max(0, -v_offset)
            new_u0 = max(0, u_offset)
            new_v0 = max(0, v_offset)
            width = min(old.width - old_u0, geometry.width - new_u0)
            height = min(old.height - old_v0, geometry.height - new_v0)
            if width > 0 and height > 0:
                new_sum[new_v0 : new_v0 + height, new_u0 : new_u0 + width] = self._sum[
                    old_v0 : old_v0 + height, old_u0 : old_u0 + width
                ]
                new_weight[new_v0 : new_v0 + height, new_u0 : new_u0 + width] = (
                    self._weight[old_v0 : old_v0 + height, old_u0 : old_u0 + width]
                )
            self.geometry = geometry
            self._sum = new_sum
            self._weight = new_weight

    @staticmethod
    def _feather(mask: np.ndarray, radius: int) -> np.ndarray:
        confidence = np.clip(mask.astype(np.float32) / 255.0, 0.0, 1.0)
        valid = (confidence > 0.0).astype(np.uint8)
        if radius <= 0:
            return confidence
        distance = cv2.distanceTransform(valid, cv2.DIST_L2, 3)
        feather = np.minimum(distance / float(radius), 1.0).astype(np.float32)
        return feather * confidence

    def blend(
        self,
        bev_image: np.ndarray,
        bev_mask: np.ndarray,
        robot_pose: Pose2D,
        bev_geometry: BevGeometry,
        *,
        feather_pixels: int = 20,
        observation_weight: float = 1.0,
    ) -> int:
        if bev_image.shape[:2] != (bev_geometry.height, bev_geometry.width):
            raise ValueError("BEV image dimensions do not match BEV geometry")
        if bev_mask.shape[:2] != bev_image.shape[:2]:
            raise ValueError("BEV mask dimensions do not match image")
        with self.lock:
            source = np.asarray(
                [
                    [0.0, 0.0],
                    [bev_geometry.width - 1.0, 0.0],
                    [bev_geometry.width - 1.0, bev_geometry.height - 1.0],
                    [0.0, bev_geometry.height - 1.0],
                ],
                dtype=np.float32,
            )
            destination = bev_corners_to_map_pixels(
                bev_geometry, robot_pose, self.geometry
            ).astype(np.float32)
            homography = cv2.getPerspectiveTransform(source, destination)
            size = (self.geometry.width, self.geometry.height)
            warped_image = cv2.warpPerspective(
                bev_image, homography, size, flags=cv2.INTER_LINEAR
            ).astype(np.float32)
            warped_mask = cv2.warpPerspective(
                self._feather(bev_mask, feather_pixels),
                homography,
                size,
                flags=cv2.INTER_LINEAR,
            )
            warped_weight = np.clip(warped_mask, 0.0, 1.0) * float(observation_weight)
            valid_count = int(np.count_nonzero(warped_weight > 1e-4))
            if valid_count == 0:
                return 0
            self._sum += warped_image * warped_weight[..., None]
            self._weight += warped_weight
            return valid_count

    def snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        with self.lock:
            denominator = np.maximum(self._weight, 1e-6)
            image = np.clip(self._sum / denominator[..., None], 0, 255).astype(np.uint8)
            image[self._weight <= 1e-6] = 0
            return image, self._weight.copy()
