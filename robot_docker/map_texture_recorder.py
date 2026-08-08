#!/usr/bin/env python3
"""Project downward-looking camera samples onto a saved SLAM occupancy map.

The result is a human-readable visual layer only. Navigation and click safety
continue to use the original occupancy PGM.
"""

from __future__ import annotations

import ast
import json
import math
import os
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class SavedMapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


@dataclass(frozen=True)
class CameraSample:
    jpeg: bytes
    x: float
    y: float
    yaw: float


def world_to_map_pixel(x: float, y: float, info: SavedMapInfo) -> tuple[float, float]:
    dx = x - info.origin_x
    dy = y - info.origin_y
    cosine = math.cos(info.origin_yaw)
    sine = math.sin(info.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        local_x / info.resolution - 0.5,
        info.height - local_y / info.resolution - 0.5,
    )


def _world_ground_point(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    forward: float,
    left: float,
) -> tuple[float, float]:
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    return (
        robot_x + cosine * forward - sine * left,
        robot_y + sine * forward + cosine * left,
    )


def load_saved_map(map_output: str) -> tuple[np.ndarray, SavedMapInfo]:
    image = cv2.imread(f"{map_output}.pgm", cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"saved occupancy image is unavailable: {map_output}.pgm")
    metadata: dict[str, str] = {}
    with open(f"{map_output}.yaml", "r", encoding="utf-8") as yaml_file:
        for raw_line in yaml_file:
            line = raw_line.split("#", 1)[0].strip()
            if line and ":" in line:
                key, value = (part.strip() for part in line.split(":", 1))
                metadata[key] = value
    origin = ast.literal_eval(metadata.get("origin", "[0, 0, 0]"))
    height, width = image.shape
    return image, SavedMapInfo(
        width=width,
        height=height,
        resolution=float(metadata["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
    )


def compose_texture(
    occupancy: np.ndarray,
    info: SavedMapInfo,
    samples: list[CameraSample],
    *,
    near_m: float = 0.25,
    far_m: float = 3.5,
    near_width_m: float = 1.25,
    far_width_m: float = 2.6,
    source_top_fraction: float = 0.42,
) -> np.ndarray:
    """Blend camera ground trapezoids into occupancy-map pixel coordinates."""
    canvas_sum = np.zeros((info.height, info.width, 3), dtype=np.float32)
    canvas_weight = np.zeros((info.height, info.width), dtype=np.float32)

    for sample in samples:
        encoded = np.frombuffer(sample.jpeg, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            continue
        frame_height, frame_width = frame.shape[:2]
        source = np.float32(
            [
                [frame_width * 0.27, frame_height * source_top_fraction],
                [frame_width * 0.73, frame_height * source_top_fraction],
                [frame_width * 0.98, frame_height * 0.98],
                [frame_width * 0.02, frame_height * 0.98],
            ]
        )
        ground_corners = [
            (far_m, far_width_m / 2.0),
            (far_m, -far_width_m / 2.0),
            (near_m, -near_width_m / 2.0),
            (near_m, near_width_m / 2.0),
        ]
        destination = np.float32(
            [
                world_to_map_pixel(
                    *_world_ground_point(
                        sample.x, sample.y, sample.yaw, forward, left
                    ),
                    info,
                )
                for forward, left in ground_corners
            ]
        )
        left = max(0, int(math.floor(float(destination[:, 0].min()))) - 2)
        top = max(0, int(math.floor(float(destination[:, 1].min()))) - 2)
        right = min(
            info.width, int(math.ceil(float(destination[:, 0].max()))) + 3
        )
        bottom = min(
            info.height, int(math.ceil(float(destination[:, 1].max()))) + 3
        )
        if right <= left or bottom <= top:
            continue
        local_destination = destination - np.float32([left, top])
        homography = cv2.getPerspectiveTransform(source, local_destination)
        source_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv2.fillConvexPoly(source_mask, source.astype(np.int32), 255)
        warped = cv2.warpPerspective(
            frame, homography, (right - left, bottom - top), flags=cv2.INTER_LINEAR
        )
        weight = cv2.warpPerspective(
            source_mask,
            homography,
            (right - left, bottom - top),
            flags=cv2.INTER_LINEAR,
        ).astype(np.float32) / 255.0
        # Feather boundaries to avoid hard quadrilateral seams.
        weight = cv2.GaussianBlur(weight, (0, 0), sigmaX=1.2)
        canvas_sum[top:bottom, left:right] += (
            warped.astype(np.float32) * weight[..., None]
        )
        canvas_weight[top:bottom, left:right] += weight

    base = cv2.cvtColor(occupancy, cv2.COLOR_GRAY2BGR)
    observed = canvas_weight > 0.08
    # Unknown/occupied geometry remains visible and authoritative.
    usable = observed & (occupancy >= 250)
    if np.any(usable):
        averaged = canvas_sum / np.maximum(canvas_weight[..., None], 0.001)
        base[usable] = np.clip(averaged[usable], 0, 255).astype(np.uint8)
    return base


def save_texture_atomic(path: str, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("camera texture PNG encoding failed")
    temporary = f"{path}.tmp"
    with open(temporary, "wb") as output_file:
        output_file.write(encoded.tobytes())
    os.replace(temporary, path)


class MapTextureRecorder(Node):
    def __init__(self) -> None:
        super().__init__("map_texture_recorder")
        self.declare_parameter("camera_url", "http://127.0.0.1:8080/stream.mjpg")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("sample_period", 0.25)
        self.declare_parameter("minimum_sample_distance", 0.08)
        self.declare_parameter("minimum_sample_angle", 0.20)
        self.declare_parameter("maximum_samples", 600)
        # Ground-plane calibration knobs. They describe the visible trapezoid
        # for the current fixed camera tilt and can be tuned without code edits.
        self.declare_parameter("projection_near_m", 0.18)
        self.declare_parameter("projection_far_m", 2.0)
        self.declare_parameter("projection_near_width_m", 0.85)
        self.declare_parameter("projection_far_width_m", 1.8)
        self.declare_parameter("projection_source_top_fraction", 0.50)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, "/autonomous_mapping/status", self._on_mapping_status, qos
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame: np.ndarray | None = None
        self._samples: list[CameraSample] = []
        self._mapping_enabled = False
        self._last_pose: tuple[float, float, float] | None = None
        self._last_saved_map = ""
        self._last_save_sequence = -1
        self._building = False
        self.create_timer(
            float(self.get_parameter("sample_period").value), self._sample
        )
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self.get_logger().info("camera map texture recorder ready")

    def _capture_loop(self) -> None:
        camera_url = str(self.get_parameter("camera_url").value)
        while not self._stop_event.is_set():
            capture = cv2.VideoCapture(camera_url)
            if not capture.isOpened():
                capture.release()
                self._stop_event.wait(2.0)
                continue
            while not self._stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with self._lock:
                    self._latest_frame = frame
            capture.release()
            self._stop_event.wait(0.5)

    def _on_mapping_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        enabled = bool(status.get("enabled", False))
        if enabled and not self._mapping_enabled:
            with self._lock:
                self._samples.clear()
                self._last_pose = None
            self.get_logger().info("camera sampling started for new mapping session")
        self._mapping_enabled = enabled
        saved_map = str(status.get("saved_map") or "")
        save_sequence = int(status.get("save_sequence", 0))
        is_new_save = save_sequence > self._last_save_sequence or (
            save_sequence == 0 and saved_map != self._last_saved_map
        )
        if saved_map and is_new_save and not self._building:
            self._last_saved_map = saved_map
            self._last_save_sequence = save_sequence
            self._building = True
            threading.Thread(
                target=self._build_texture, args=(saved_map,), daemon=True
            ).start()

    def _sample(self) -> None:
        if not self._mapping_enabled:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        pose = (float(translation.x), float(translation.y), float(yaw))
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            previous = self._last_pose
        if frame is None:
            return
        if previous is not None:
            moved = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
            turned = abs(
                math.atan2(
                    math.sin(pose[2] - previous[2]),
                    math.cos(pose[2] - previous[2]),
                )
            )
            if moved < float(
                self.get_parameter("minimum_sample_distance").value
            ) and turned < float(self.get_parameter("minimum_sample_angle").value):
                return
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if not ok:
            return
        sample = CameraSample(encoded.tobytes(), *pose)
        with self._lock:
            maximum = int(self.get_parameter("maximum_samples").value)
            if len(self._samples) >= maximum:
                self._samples.pop(0)
            self._samples.append(sample)
            self._last_pose = pose

    def _build_texture(self, map_output: str) -> None:
        try:
            # Map saver announces success immediately before this callback is seen.
            time.sleep(0.25)
            with self._lock:
                samples = list(self._samples)
            if not samples:
                self.get_logger().warn(
                    "no camera samples collected; keeping the LiDAR-only map"
                )
                return
            occupancy, info = load_saved_map(map_output)
            texture = compose_texture(
                occupancy,
                info,
                samples,
                near_m=float(self.get_parameter("projection_near_m").value),
                far_m=float(self.get_parameter("projection_far_m").value),
                near_width_m=float(
                    self.get_parameter("projection_near_width_m").value
                ),
                far_width_m=float(
                    self.get_parameter("projection_far_width_m").value
                ),
                source_top_fraction=float(
                    self.get_parameter("projection_source_top_fraction").value
                ),
            )
            output_path = f"{map_output}_texture.png"
            save_texture_atomic(output_path, texture)
            self.get_logger().info(
                f"camera map texture saved from {len(samples)} samples: {output_path}"
            )
        except Exception as error:
            self.get_logger().error(f"camera map texture generation failed: {error}")
        finally:
            self._building = False

    def destroy_node(self):
        self._stop_event.set()
        self._capture_thread.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MapTextureRecorder()
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
