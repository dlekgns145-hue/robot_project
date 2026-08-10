#!/usr/bin/env python3
"""Project downward-looking camera samples onto a saved SLAM occupancy map.

The result is a human-readable visual layer only. Navigation and click safety
continue to use the original occupancy PGM.
"""

from __future__ import annotations

import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from map_texture_core import (
    CameraSample,
    compose_visual_layers,
    load_saved_map,
    save_json_atomic,
    save_texture_atomic,
)


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
            texture, obstacle_texture, materials = compose_visual_layers(
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
            obstacle_output_path = f"{map_output}_obstacles.png"
            materials_output_path = f"{map_output}_materials.json"
            save_texture_atomic(output_path, texture)
            save_texture_atomic(obstacle_output_path, obstacle_texture)
            save_json_atomic(materials_output_path, materials)
            self.get_logger().info(
                "camera map layers saved from "
                f"{len(samples)} samples: {output_path}; "
                f"obstacle material={materials['dominant']} "
                f"({materials['dominant_share']:.0%})"
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
