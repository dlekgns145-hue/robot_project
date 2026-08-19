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


def estimate_component_height_m(
    *,
    image_height: int,
    top_y: int,
    contact_y: int,
    camera_height_m: float = 0.24,
    camera_pitch_down_deg: float = 18.0,
    vertical_fov_deg: float = 50.0,
) -> float:
    """Estimate upright obstacle height from its top and floor-contact pixels.

    This deliberately errs toward blocking: invalid geometry returns infinity,
    so only clearly shallow floor protrusions can be classified traversable.
    """

    if image_height <= 1 or not 1.0 < vertical_fov_deg < 179.0:
        return math.inf
    if camera_height_m <= 0.0 or not 0 <= top_y <= contact_y < image_height:
        return math.inf
    focal_y = (image_height / 2.0) / math.tan(
        math.radians(vertical_fov_deg) / 2.0
    )
    center_y = image_height / 2.0
    pitch = math.radians(camera_pitch_down_deg)
    contact_angle = pitch + math.atan((contact_y - center_y) / focal_y)
    if contact_angle <= math.radians(1.0) or contact_angle >= math.radians(89.0):
        return math.inf
    distance = camera_height_m / math.tan(contact_angle)
    top_angle = pitch + math.atan((top_y - center_y) / focal_y)
    estimated = camera_height_m - distance * math.tan(top_angle)
    if not math.isfinite(estimated) or estimated < 0.0:
        return math.inf
    return estimated


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
    maximum_traversable_height_m: float = 0.025,
    camera_height_m: float = 0.24,
    camera_pitch_down_deg: float = 18.0,
    vertical_fov_deg: float = 50.0,
    reflection_luminance_min: float = 190.0,
    reflection_chroma_max: float = 12.0,
    minimum_fill_ratio: float = 0.4,
    wide_achromatic_width_fraction: float = 0.65,
    wide_achromatic_chroma_max: float = 5.0,
    debug_sink: list[dict] | None = None,
) -> tuple[list[float], int]:
    """Estimate obstacle ranges, ordered from image-left to image-right.

    A Lab colour model is sampled from two patches near the robot.  Requiring a
    connected component of useful size rejects isolated soil/grass texture.
    Range is an inverse-perspective estimate of the component's ground contact.

    A glossy/reflective floor can mirror an overhead light into a patch that
    reads as bright and different enough from the floor reference to pass the
    thresholds above, yet it is flat -- physically 0 m tall, not a raised
    object. Such a highlight is near-white (low absolute chroma) and much
    brighter than any matte floor or object seen indoors, unlike a real
    object's more saturated colour, so components matching that signature are
    rejected before height estimation runs.

    A shadow (the robot's own, or one cast across the floor) and floor
    texture/reflection patterns are the other common false positive: they are
    fragmented -- speckled or streaky -- rather than the compact, mostly-solid
    silhouette a real raised object leaves in the mask, even when they span a
    wide bounding box. ``area / (width * height)`` catches both regardless of
    whether the pattern is bright or dark.

    A shadow can also be dense enough to clear that fill-ratio gate (a hard
    shadow edge, not a speckled one). What still sets it apart from a real
    object is width: a shadow or lighting gradient runs most of the way
    across the visible floor, while an object the robot actually needs to
    avoid occupies a bounded part of it -- and both a reflection and a
    shadow are close to colourless (low chroma) where a real object usually
    is not. A component that is both wide and essentially colourless is
    rejected regardless of how bright or dark it is.

    When ``debug_sink`` is given, every candidate component past the
    area/height gate appends a dict of its measurements and outcome
    (``"accepted"``, ``"rejected_reflection"``, or ``"rejected_traversable"``)
    -- this is what a live capture can't show, since a photo alone doesn't
    reveal which pixels the algorithm grouped together or why.
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
        component_mask = labels[y : y + component_height, x : x + component_width] == label
        component_lab = lab[y : y + component_height, x : x + component_width][component_mask]
        mean_luminance = float(component_lab[:, 0].mean())
        mean_chroma = float(np.linalg.norm(component_lab[:, 1:3].mean(axis=0) - 128.0))
        contact_y = min(bottom, y + component_height - 1)
        vertical = (contact_y - top) / max(1, bottom - top)
        vertical = max(0.0, min(1.0, vertical))
        distance = near_m + (far_m - near_m) * ((1.0 - vertical) ** 1.35)
        fill_ratio = float(area) / max(1, component_width * component_height)
        debug_entry = {
            "bbox": [int(x), int(y), int(component_width), int(component_height)],
            "area": int(area),
            "fill_ratio": round(fill_ratio, 2),
            "mean_luminance": round(mean_luminance, 1),
            "mean_chroma": round(mean_chroma, 1),
            "distance_m": round(distance, 3),
        }
        if fill_ratio < minimum_fill_ratio:
            if debug_sink is not None:
                debug_entry["status"] = "rejected_sparse_pattern"
                debug_sink.append(debug_entry)
            continue
        if (
            component_width >= wide_achromatic_width_fraction * width
            and mean_chroma <= wide_achromatic_chroma_max
        ):
            if debug_sink is not None:
                debug_entry["status"] = "rejected_wide_achromatic"
                debug_sink.append(debug_entry)
            continue
        if mean_luminance >= reflection_luminance_min and mean_chroma <= reflection_chroma_max:
            if debug_sink is not None:
                debug_entry["status"] = "rejected_reflection"
                debug_sink.append(debug_entry)
            continue
        estimated_height = estimate_component_height_m(
            image_height=height,
            top_y=int(y),
            contact_y=int(contact_y),
            camera_height_m=camera_height_m,
            camera_pitch_down_deg=camera_pitch_down_deg,
            vertical_fov_deg=vertical_fov_deg,
        )
        debug_entry["estimated_height_m"] = (
            None if math.isinf(estimated_height) else round(estimated_height, 4)
        )
        if 0.0 <= estimated_height <= maximum_traversable_height_m:
            if debug_sink is not None:
                debug_entry["status"] = "rejected_traversable"
                debug_sink.append(debug_entry)
            continue
        first_ray = max(0, int(math.floor(x / width * ray_count)))
        last_ray = min(
            ray_count - 1,
            int(math.ceil((x + component_width) / width * ray_count)) - 1,
        )
        for ray in range(first_ray, last_ray + 1):
            ranges[ray] = min(ranges[ray], distance)
        accepted += 1
        if debug_sink is not None:
            debug_entry["status"] = "accepted"
            debug_sink.append(debug_entry)
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
        # servo_s2=-65 / 18 deg is the one validated pair (many hours of
        # correct real-obstacle detection on 2026-08-18). A later attempt at
        # servo_s2=-90 with a geometrically *estimated* 25 deg / 0.15 (no
        # physical measurement, no second calibrated data point) started
        # flagging floor seams/marks as ~5 cm obstacles that LiDAR and the
        # live camera feed both showed were clear -- reverted. Do not change
        # tilt again without either a real measurement or a second
        # independently-verified calibration pair.
        self.declare_parameter("source_top_fraction", 0.50)
        self.declare_parameter("source_bottom_fraction", 0.94)
        self.declare_parameter("color_threshold", 20.0)
        self.declare_parameter("luminance_threshold", 38.0)
        self.declare_parameter("minimum_component_area", 150)
        self.declare_parameter("minimum_component_height", 12)
        self.declare_parameter("maximum_traversable_height_m", 0.025)
        self.declare_parameter("camera_height_m", 0.24)
        self.declare_parameter("camera_pitch_down_deg", 18.0)
        self.declare_parameter("vertical_fov_deg", 50.0)
        self.declare_parameter("reflection_luminance_min", 190.0)
        self.declare_parameter("reflection_chroma_max", 12.0)
        self.declare_parameter("minimum_fill_ratio", 0.4)
        self.declare_parameter("wide_achromatic_width_fraction", 0.65)
        self.declare_parameter("wide_achromatic_chroma_max", 5.0)
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
        debug_sink: list[dict] = []
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
                maximum_traversable_height_m=float(
                    self.get_parameter("maximum_traversable_height_m").value
                ),
                camera_height_m=float(self.get_parameter("camera_height_m").value),
                camera_pitch_down_deg=float(
                    self.get_parameter("camera_pitch_down_deg").value
                ),
                vertical_fov_deg=float(
                    self.get_parameter("vertical_fov_deg").value
                ),
                reflection_luminance_min=float(
                    self.get_parameter("reflection_luminance_min").value
                ),
                reflection_chroma_max=float(
                    self.get_parameter("reflection_chroma_max").value
                ),
                minimum_fill_ratio=float(
                    self.get_parameter("minimum_fill_ratio").value
                ),
                wide_achromatic_width_fraction=float(
                    self.get_parameter("wide_achromatic_width_fraction").value
                ),
                wide_achromatic_chroma_max=float(
                    self.get_parameter("wide_achromatic_chroma_max").value
                ),
                debug_sink=debug_sink,
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
        nearest_debug = (
            min(debug_sink, key=lambda entry: entry["distance_m"])
            if debug_sink
            else None
        )
        self._publish_status(True, components, nearest, frame_age, nearest_debug)

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
        self,
        connected: bool,
        components: int,
        nearest: float | None,
        age: float,
        nearest_debug: dict | None = None,
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "connected": connected,
                "components": components,
                "nearest_m": nearest,
                "frame_age_s": round(age, 3),
                "nearest_candidate": nearest_debug,
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
