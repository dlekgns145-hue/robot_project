#!/usr/bin/env python3
"""Continuously fuse robot camera pixels with LiDAR-confirmed map obstacles.

Camera appearance is a visual layer only.  SLAM/LiDAR occupancy remains the
navigation and click-safety source of truth at all times.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import cv2
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from map_texture_core import (
    SavedMapInfo,
    load_saved_map,
    save_image_atomic,
    save_json_atomic,
    save_texture_atomic,
    summarize_obstacle_materials,
)
from obstacle_texture_fusion import (
    CameraLidarCalibration,
    ObstacleTextureAccumulator,
    observations_from_scan,
    occupancy_grid_to_pgm,
    render_obstacle_texture,
)


class MapTextureRecorder(Node):
    def __init__(self) -> None:
        super().__init__("map_texture_recorder")
        self.declare_parameter("camera_url", "http://127.0.0.1:8080/stream.mjpg")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_output", "/opt/robot-control/maps/orchard_map")
        self.declare_parameter("sample_period", 0.20)
        self.declare_parameter("live_render_period", 2.0)
        self.declare_parameter("maximum_frame_age", 0.75)
        self.declare_parameter("maximum_scan_age", 0.75)
        self.declare_parameter("maximum_sensor_skew", 0.18)
        self.declare_parameter("accumulator_cell_size_m", 0.04)
        self.declare_parameter("maximum_obstacle_cells", 50000)
        self.declare_parameter("endpoint_tolerance_pixels", 2)
        self.declare_parameter("paint_radius_pixels", 1)
        self.declare_parameter("camera_horizontal_fov_deg", 68.0)
        self.declare_parameter("camera_vertical_fov_deg", 50.0)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_pitch_down_deg", 18.0)
        self.declare_parameter("camera_height_m", 0.24)
        self.declare_parameter("camera_sample_height_fraction", 0.16)
        self.declare_parameter("camera_sample_half_width_pixels", 3)
        self.declare_parameter("lidar_x_offset_m", -0.0046)
        self.declare_parameter("lidar_y_offset_m", 0.0)
        self.declare_parameter("scan_stride", 2)
        # Keep the old names declared so an r5/r6 launch file can start this
        # upgraded recorder during a rolling deployment. They are intentionally
        # unused by the LiDAR-endpoint fusion algorithm.
        self.declare_parameter("projection_near_m", 0.18)
        self.declare_parameter("projection_far_m", 2.0)
        self.declare_parameter("projection_near_width_m", 0.85)
        self.declare_parameter("projection_far_width_m", 1.8)
        self.declare_parameter("projection_source_top_fraction", 0.50)

        self._calibration = CameraLidarCalibration(
            horizontal_fov_deg=float(
                self.get_parameter("camera_horizontal_fov_deg").value
            ),
            vertical_fov_deg=float(
                self.get_parameter("camera_vertical_fov_deg").value
            ),
            camera_yaw_offset_deg=float(
                self.get_parameter("camera_yaw_offset_deg").value
            ),
            camera_pitch_down_deg=float(
                self.get_parameter("camera_pitch_down_deg").value
            ),
            camera_height_m=float(self.get_parameter("camera_height_m").value),
            lidar_x_offset_m=float(self.get_parameter("lidar_x_offset_m").value),
            lidar_y_offset_m=float(self.get_parameter("lidar_y_offset_m").value),
            sample_height_fraction=float(
                self.get_parameter("camera_sample_height_fraction").value
            ),
            sample_half_width_pixels=int(
                self.get_parameter("camera_sample_half_width_pixels").value
            ),
            scan_stride=int(self.get_parameter("scan_stride").value),
        )
        self._calibration.validate()
        self._accumulator = ObstacleTextureAccumulator(
            cell_size_m=float(
                self.get_parameter("accumulator_cell_size_m").value
            ),
            maximum_cells=int(self.get_parameter("maximum_obstacle_cells").value),
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, "/autonomous_mapping/status", self._on_mapping_status, status_qos
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()
        self._render_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_frame_at = 0.0
        self._latest_scan = None
        self._latest_scan_at = 0.0
        self._scan_sequence = 0
        self._last_sampled_scan_sequence = -1
        self._latest_map = None
        self._map_sequence = 0
        self._mapping_enabled = False
        self._last_saved_map = ""
        self._last_save_sequence = -1
        self._building = False
        self._accepted_observations = 0

        self.create_timer(float(self.get_parameter("sample_period").value), self._sample)
        self.create_timer(
            float(self.get_parameter("live_render_period").value),
            self._schedule_live_render,
        )
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self.get_logger().info(
            "LiDAR-camera obstacle texture fusion ready; occupancy remains LiDAR-only"
        )

    def _capture_loop(self) -> None:
        camera_url = str(self.get_parameter("camera_url").value)
        while not self._stop_event.is_set():
            capture = cv2.VideoCapture(camera_url)
            if not capture.isOpened():
                capture.release()
                self._stop_event.wait(2.0)
                continue
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while not self._stop_event.is_set():
                ok, frame = capture.read()
                if not ok:
                    break
                with self._lock:
                    self._latest_frame = frame
                    self._latest_frame_at = time.monotonic()
            capture.release()
            self._stop_event.wait(0.5)

    def _on_scan(self, message: LaserScan) -> None:
        with self._lock:
            self._latest_scan = message
            self._latest_scan_at = time.monotonic()
            self._scan_sequence += 1

    def _on_map(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._latest_map = message
            self._map_sequence += 1

    def _on_mapping_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        enabled = bool(status.get("enabled", False))
        if enabled and not self._mapping_enabled:
            with self._lock:
                self._accumulator.clear()
                self._accepted_observations = 0
                self._last_sampled_scan_sequence = -1
            self.get_logger().info(
                "new mapping session: LiDAR-camera obstacle accumulation started"
            )
        if self._mapping_enabled and not enabled:
            self._write_live_status(active=False)
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
                target=self._build_final_layers, args=(saved_map,), daemon=True
            ).start()

    def _robot_pose(self, stamp=None) -> tuple[float, float, float]:
        transform_time = Time() if stamp is None else Time.from_msg(stamp)
        transform = self.tf_buffer.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("base_frame").value),
            transform_time,
            timeout=Duration(seconds=0.15),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), float(yaw)

    def _sample(self) -> None:
        if not self._mapping_enabled:
            return
        now = time.monotonic()
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            frame_at = self._latest_frame_at
            scan = self._latest_scan
            scan_at = self._latest_scan_at
            scan_sequence = self._scan_sequence
            already_sampled = scan_sequence == self._last_sampled_scan_sequence
        if frame is None or scan is None or already_sampled:
            return
        if now - frame_at > float(self.get_parameter("maximum_frame_age").value):
            return
        if now - scan_at > float(self.get_parameter("maximum_scan_age").value):
            return
        if abs(frame_at - scan_at) > float(
            self.get_parameter("maximum_sensor_skew").value
        ):
            return
        try:
            robot_x, robot_y, robot_yaw = self._robot_pose(scan.header.stamp)
        except TransformException:
            return
        try:
            observations = observations_from_scan(
                frame,
                scan.ranges,
                angle_min=float(scan.angle_min),
                angle_increment=float(scan.angle_increment),
                range_min=float(scan.range_min),
                range_max=float(scan.range_max),
                robot_x=robot_x,
                robot_y=robot_y,
                robot_yaw=robot_yaw,
                calibration=self._calibration,
            )
        except ValueError as error:
            self.get_logger().warn(f"LiDAR-camera sample rejected: {error}")
            return
        with self._lock:
            accepted = self._accumulator.add(observations)
            self._accepted_observations += accepted
            self._last_sampled_scan_sequence = scan_sequence

    @staticmethod
    def _saved_map_info(message: OccupancyGrid) -> SavedMapInfo:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return SavedMapInfo(
            width=int(message.info.width),
            height=int(message.info.height),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=yaw,
        )

    def _render(self, occupancy, info, observations):
        base, obstacle_layer, fusion = render_obstacle_texture(
            occupancy,
            info,
            observations,
            endpoint_tolerance_pixels=int(
                self.get_parameter("endpoint_tolerance_pixels").value
            ),
            paint_radius_pixels=int(self.get_parameter("paint_radius_pixels").value),
        )
        materials = summarize_obstacle_materials(
            fusion.pop("camera_bgr"), fusion.pop("observed_mask")
        )
        materials.update(
            {
                "fusion_method": "lidar-endpoint-camera-pixel-v1",
                "observation_cells": fusion["observation_cells"],
                "total_hits": fusion["total_hits"],
                "navigation_safe": False,
                "occupancy_source": "lidar_slam_only",
            }
        )
        return base, obstacle_layer, materials

    def _schedule_live_render(self) -> None:
        if not self._mapping_enabled or self._render_lock.locked():
            return
        threading.Thread(target=self._render_live, daemon=True).start()

    def _render_live(self) -> None:
        if not self._render_lock.acquire(blocking=False):
            return
        try:
            with self._lock:
                map_message = self._latest_map
                map_sequence = self._map_sequence
                observations = self._accumulator.observations()
                accepted = self._accepted_observations
            if map_message is None or not observations:
                return
            occupancy = occupancy_grid_to_pgm(
                map_message.data,
                int(map_message.info.width),
                int(map_message.info.height),
            )
            info = self._saved_map_info(map_message)
            base, obstacle_layer, materials = self._render(
                occupancy, info, observations
            )
            map_output = str(self.get_parameter("map_output").value)
            live_output = f"{map_output}_live"
            save_image_atomic(f"{live_output}.pgm", occupancy, extension=".pgm")
            self._save_live_yaml(live_output, info)
            save_texture_atomic(f"{live_output}_texture.png", base)
            save_texture_atomic(f"{live_output}_obstacles.png", obstacle_layer)
            save_json_atomic(f"{live_output}_materials.json", materials)
            self._write_live_status(
                active=True,
                map_sequence=map_sequence,
                observation_cells=len(observations),
                accepted_observations=accepted,
            )
        except Exception as error:
            self.get_logger().error(f"live obstacle texture generation failed: {error}")
        finally:
            self._render_lock.release()

    @staticmethod
    def _save_live_yaml(map_output: str, info: SavedMapInfo) -> None:
        path = f"{map_output}.yaml"
        temporary = f"{path}.tmp"
        image_name = os.path.basename(f"{map_output}.pgm")
        content = (
            f"image: {image_name}\n"
            "mode: trinary\n"
            f"resolution: {info.resolution:.12g}\n"
            f"origin: [{info.origin_x:.12g}, {info.origin_y:.12g}, {info.origin_yaw:.12g}]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
        with open(temporary, "w", encoding="utf-8") as output_file:
            output_file.write(content)
        os.replace(temporary, path)

    def _write_live_status(self, *, active: bool, **extra) -> None:
        map_output = str(self.get_parameter("map_output").value)
        payload = {
            "schema": "robot-live-obstacle-texture-v1",
            "active": bool(active),
            "updated_unix": time.time(),
            "navigation_safe": False,
            "occupancy_source": "lidar_slam_only",
            **extra,
        }
        try:
            save_json_atomic(f"{map_output}_live_status.json", payload)
        except OSError as error:
            self.get_logger().warn(f"live texture status write failed: {error}")

    def _build_final_layers(self, map_output: str) -> None:
        try:
            time.sleep(0.30)
            with self._lock:
                observations = self._accumulator.observations()
            if not observations:
                self.get_logger().warn(
                    "no synchronized LiDAR-camera observations; keeping LiDAR-only map"
                )
                return
            occupancy, info = load_saved_map(map_output)
            base, obstacle_layer, materials = self._render(
                occupancy, info, observations
            )
            save_texture_atomic(f"{map_output}_texture.png", base)
            save_texture_atomic(f"{map_output}_obstacles.png", obstacle_layer)
            save_json_atomic(f"{map_output}_materials.json", materials)
            self.get_logger().info(
                "final LiDAR-camera obstacle layer saved from "
                f"{len(observations)} world cells; textured pixels="
                f"{materials['observed_pixels']}"
            )
        except Exception as error:
            self.get_logger().error(f"final obstacle texture generation failed: {error}")
        finally:
            self._building = False

    def destroy_node(self):
        self._stop_event.set()
        self._write_live_status(active=False)
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
