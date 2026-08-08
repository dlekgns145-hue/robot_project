#!/usr/bin/env python3
"""Publish conservative near-field camera obstacles as a synthetic LaserScan.

The scan is consumed only by Nav2's rolling local costmap.  It is intentionally
not sent to slam_toolbox: monocular distance is approximate and must never
become a permanent wall in the saved LiDAR map.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Sequence

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def obstacle_ranges_from_frame(
    frame: np.ndarray,
    *,
    ray_count: int = 61,
    near_m: float = 0.22,
    far_m: float = 2.0,
    source_top_fraction: float = 0.50,
    source_bottom_fraction: float = 0.94,
    color_threshold: float = 20.0,
    luminance_threshold: float = 38.0,
    minimum_component_area: int = 150,
    minimum_component_height: int = 12,
) -> tuple[list[float], int]:
    """Estimate obstacle ranges, ordered from image-left to image-right.

    A Lab colour model is sampled from two patches near the robot.  Requiring a
    connected component of useful size rejects isolated soil/grass texture.
    Range is an inverse-perspective estimate of the component's ground contact.
    """

    if frame is None or frame.ndim != 3 or frame.shape[0] < 40 or frame.shape[1] < 40:
        raise ValueError("camera frame is missing or too small")
    ray_count = max(3, int(ray_count))
    height, width = frame.shape[:2]
    top = max(0, min(height - 2, int(height * source_top_fraction)))
    bottom = max(top + 1, min(height - 1, int(height * source_bottom_fraction)))

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB).astype(np.float32)
    roi = np.zeros((height, width), dtype=np.uint8)
    polygon = np.array(
        [
            [int(width * 0.27), top],
            [int(width * 0.73), top],
            [int(width * 0.96), bottom],
            [int(width * 0.04), bottom],
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(roi, polygon, 255)

    reference_y0 = max(top, bottom - max(8, int(height * 0.07)))
    patches = np.concatenate(
        [
            lab[reference_y0:bottom, int(width * 0.10):int(width * 0.32)].reshape(-1, 3),
            lab[reference_y0:bottom, int(width * 0.68):int(width * 0.90)].reshape(-1, 3),
        ],
        axis=0,
    )
    if patches.size == 0:
        raise ValueError("camera floor reference is empty")
    reference = np.median(patches, axis=0)
    chroma_delta = np.linalg.norm(lab[:, :, 1:3] - reference[1:3], axis=2)
    luminance_delta = np.abs(lab[:, :, 0] - reference[0])
    candidate = np.where(
        ((chroma_delta >= color_threshold) | (luminance_delta >= luminance_threshold))
        & (roi > 0),
        255,
        0,
    ).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel)
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel, iterations=2)

    component_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        candidate, connectivity=8
    )
    ranges = [math.inf] * ray_count
    accepted = 0
    for label in range(1, component_count):
        x, y, component_width, component_height, area = stats[label]
        if area < minimum_component_area or component_height < minimum_component_height:
            continue
        contact_y = min(bottom, y + component_height - 1)
        vertical = (contact_y - top) / max(1, bottom - top)
        vertical = max(0.0, min(1.0, vertical))
        distance = near_m + (far_m - near_m) * ((1.0 - vertical) ** 1.35)
        first_ray = max(0, int(math.floor(x / width * ray_count)))
        last_ray = min(
            ray_count - 1,
            int(math.ceil((x + component_width) / width * ray_count)) - 1,
        )
        for ray in range(first_ray, last_ray + 1):
            ranges[ray] = min(ranges[ray], distance)
        accepted += 1
    return ranges, accepted


def temporally_confirm_ranges(
    history: Sequence[Sequence[float]], *, minimum_hits: int = 2
) -> list[float]:
    """Return median ranges observed in at least ``minimum_hits`` frames."""

    if not history:
        return []
    width = len(history[-1])
    if any(len(frame_ranges) != width for frame_ranges in history):
        raise ValueError("camera range history widths do not match")
    confirmed: list[float] = []
    for index in range(width):
        finite = sorted(
            float(frame_ranges[index])
            for frame_ranges in history
            if math.isfinite(frame_ranges[index])
        )
        if len(finite) < minimum_hits:
            confirmed.append(math.inf)
        else:
            middle = len(finite) // 2
            if len(finite) % 2:
                confirmed.append(finite[middle])
            else:
                confirmed.append((finite[middle - 1] + finite[middle]) / 2.0)
    return confirmed


class CameraObstacleGuard(Node):
    def __init__(self) -> None:
        super().__init__("camera_obstacle_guard")
        self.declare_parameter("camera_url", "http://127.0.0.1:8080/stream.mjpg")
        self.declare_parameter("output_topic", "/camera_scan")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("field_of_view_degrees", 70.0)
        self.declare_parameter("ray_count", 61)
        self.declare_parameter("near_m", 0.22)
        self.declare_parameter("far_m", 2.0)
        self.declare_parameter("source_top_fraction", 0.50)
        self.declare_parameter("source_bottom_fraction", 0.94)
        self.declare_parameter("color_threshold", 20.0)
        self.declare_parameter("luminance_threshold", 38.0)
        self.declare_parameter("minimum_component_area", 150)
        self.declare_parameter("minimum_component_height", 12)
        self.declare_parameter("temporal_window", 3)
        self.declare_parameter("temporal_minimum_hits", 2)

        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter("output_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, "/camera_obstacle_guard/status", 10
        )
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame: np.ndarray | None = None
        self._latest_frame_at = 0.0
        window = max(1, int(self.get_parameter("temporal_window").value))
        self._history: deque[list[float]] = deque(maxlen=window)
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        frequency = max(1.0, float(self.get_parameter("publish_hz").value))
        self.create_timer(1.0 / frequency, self._process_frame)
        self.get_logger().info(
            "camera obstacle guard ready; output is local-costmap only"
        )

    def _capture_loop(self) -> None:
        camera_url = str(self.get_parameter("camera_url").value)
        while not self._stop_event.is_set():
            capture = cv2.VideoCapture(camera_url)
            if not capture.isOpened():
                capture.release()
                self._stop_event.wait(1.0)
                continue
            while not self._stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with self._lock:
                    self._latest_frame = frame
                    self._latest_frame_at = time.monotonic()
            capture.release()
            self._stop_event.wait(0.3)

    def _process_frame(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            frame_age = time.monotonic() - self._latest_frame_at
        if frame is None or frame_age > 1.0:
            self._publish_status(False, 0, None, frame_age)
            return
        try:
            ranges, components = obstacle_ranges_from_frame(
                frame,
                ray_count=int(self.get_parameter("ray_count").value),
                near_m=float(self.get_parameter("near_m").value),
                far_m=float(self.get_parameter("far_m").value),
                source_top_fraction=float(
                    self.get_parameter("source_top_fraction").value
                ),
                source_bottom_fraction=float(
                    self.get_parameter("source_bottom_fraction").value
                ),
                color_threshold=float(self.get_parameter("color_threshold").value),
                luminance_threshold=float(
                    self.get_parameter("luminance_threshold").value
                ),
                minimum_component_area=int(
                    self.get_parameter("minimum_component_area").value
                ),
                minimum_component_height=int(
                    self.get_parameter("minimum_component_height").value
                ),
            )
        except (ValueError, cv2.error) as error:
            self.get_logger().warn(f"camera obstacle frame rejected: {error}")
            return
        self._history.append(ranges)
        confirmed = temporally_confirm_ranges(
            self._history,
            minimum_hits=int(
                self.get_parameter("temporal_minimum_hits").value
            ),
        )
        self._publish_scan(confirmed)
        nearest = min((value for value in confirmed if math.isfinite(value)), default=None)
        self._publish_status(True, components, nearest, frame_age)

    def _publish_scan(self, image_order_ranges: Sequence[float]) -> None:
        field_of_view = math.radians(
            float(self.get_parameter("field_of_view_degrees").value)
        )
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.angle_min = -field_of_view / 2.0
        message.angle_max = field_of_view / 2.0
        message.angle_increment = field_of_view / max(1, len(image_order_ranges) - 1)
        message.time_increment = 0.0
        message.scan_time = 1.0 / max(
            1.0, float(self.get_parameter("publish_hz").value)
        )
        message.range_min = float(self.get_parameter("near_m").value)
        message.range_max = float(self.get_parameter("far_m").value)
        # ROS angles increase toward the robot's left; image columns do the
        # opposite, so reverse the image-space bins before publishing.
        message.ranges = list(reversed(image_order_ranges))
        self.publisher.publish(message)

    def _publish_status(
        self, connected: bool, components: int, nearest: float | None, age: float
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "connected": connected,
                "components": components,
                "nearest_m": nearest,
                "frame_age_s": round(age, 3),
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)

    def destroy_node(self):
        self._stop_event.set()
        self._capture_thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CameraObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
