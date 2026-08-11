"""Camera acquisition, calibration and inverse-perspective (BEV) projection."""

from __future__ import annotations

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .coordinate_transform import flattened_points


class CameraBevNode(Node):
    """Publish a metric bird's-eye-view image and its valid-ground mask."""

    def __init__(self) -> None:
        super().__init__("camera_bev")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        self.mapping_fps = max(float(self.get_parameter("mapping_fps").value), 0.1)
        self.bev_width = int(self.get_parameter("bev_width").value)
        self.bev_height = int(self.get_parameter("bev_height").value)
        self.points_normalized = bool(self.get_parameter("points_normalized").value)
        self.publisher = self.create_publisher(
            Image, "~/bev/image", qos_profile_sensor_data
        )
        self.mask_publisher = self.create_publisher(
            Image, "~/bev/mask", qos_profile_sensor_data
        )
        self.camera_info: CameraInfo | None = None
        self.last_processed_ns = 0
        self.capture: cv2.VideoCapture | None = None
        self.capture_thread: threading.Thread | None = None
        self.capture_stop = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.latest_sequence = 0
        self.processed_sequence = -1

        if self.input_mode == "ros":
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("camera_topic").value),
                self._on_image,
                qos_profile_sensor_data,
            )
            self.get_logger().info("camera input: ROS image topic")
        elif self.input_mode == "mjpeg":
            self.capture_thread = threading.Thread(
                target=self._capture_loop, name="mjpeg-capture", daemon=True
            )
            self.capture_thread.start()
            self.create_timer(1.0 / self.mapping_fps, self._process_latest_mjpeg)
            self.get_logger().info(
                f"camera input: {self.get_parameter('camera_url').value}"
            )
        else:
            raise ValueError("input_mode must be 'ros' or 'mjpeg'")

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_mode", "ros")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_url", "http://127.0.0.1:8081/stream.mjpg")
        self.declare_parameter("mapping_fps", 3.0)
        self.declare_parameter("bev_width", 480)
        self.declare_parameter("bev_height", 640)
        self.declare_parameter("points_normalized", True)
        self.declare_parameter(
            "src_points", [0.18, 0.50, 0.82, 0.50, 0.98, 0.98, 0.02, 0.98]
        )
        self.declare_parameter("dst_points", [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0])
        self.declare_parameter(
            "roi_points", [0.18, 0.50, 0.82, 0.50, 0.98, 0.98, 0.02, 0.98]
        )
        # Empty arrays are inferred as BYTE_ARRAY by ROS 2 Humble. The
        # one-value double sentinel keeps both parameters DOUBLE_ARRAY until
        # real calibration values are supplied.
        self.declare_parameter("camera_matrix", [0.0])
        self.declare_parameter("distortion_coefficients", [0.0])
        self.declare_parameter("robot_mask_fraction", 0.08)
        self.declare_parameter("far_mask_weight", 0.20)
        self.declare_parameter("normalize_illumination", False)
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("reconnect_delay", 2.0)

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _on_image(self, message: Image) -> None:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        minimum_period = int(1_000_000_000 / self.mapping_fps)
        if stamp_ns > 0 and stamp_ns - self.last_processed_ns < minimum_period:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            self._process_frame(frame, message.header.stamp)
            self.last_processed_ns = stamp_ns
        except Exception as error:
            self.get_logger().warning(f"camera frame rejected: {error}")

    def _capture_loop(self) -> None:
        url = str(self.get_parameter("camera_url").value)
        delay = float(self.get_parameter("reconnect_delay").value)
        while not self.capture_stop.is_set():
            capture = cv2.VideoCapture(url)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.capture = capture
            if not capture.isOpened():
                self.get_logger().warning(f"cannot open MJPEG stream: {url}")
                capture.release()
                self.capture_stop.wait(delay)
                continue
            while not self.capture_stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_sequence += 1
            capture.release()
            self.capture = None
            if not self.capture_stop.is_set():
                self.get_logger().warning("MJPEG stream lost; reconnecting")
                self.capture_stop.wait(delay)

    def _process_latest_mjpeg(self) -> None:
        with self.frame_lock:
            if (
                self.latest_frame is None
                or self.latest_sequence == self.processed_sequence
            ):
                return
            frame = self.latest_frame.copy()
            self.processed_sequence = self.latest_sequence
        try:
            self._process_frame(frame, self.get_clock().now().to_msg())
        except Exception as error:
            self.get_logger().warning(f"MJPEG frame rejected: {error}")

    @staticmethod
    def _scale_points(
        points: np.ndarray, width: int, height: int, normalized: bool
    ) -> np.ndarray:
        result = points.copy().astype(np.float32)
        if normalized:
            result[:, 0] *= max(width - 1, 1)
            result[:, 1] *= max(height - 1, 1)
        return result

    def _calibration(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        if self.camera_info is not None and any(self.camera_info.k):
            return (
                np.asarray(self.camera_info.k, np.float64).reshape(3, 3),
                np.asarray(self.camera_info.d, np.float64),
            )
        matrix_values = list(self.get_parameter("camera_matrix").value)
        distortion = list(self.get_parameter("distortion_coefficients").value)
        if len(matrix_values) == 9 and distortion:
            return (
                np.asarray(matrix_values, np.float64).reshape(3, 3),
                np.asarray(distortion, np.float64),
            )
        return None, None

    def _process_frame(self, frame: np.ndarray, stamp) -> None:
        matrix, distortion = self._calibration()
        corrected = (
            cv2.undistort(frame, matrix, distortion)
            if matrix is not None and distortion is not None
            else frame
        )
        if bool(self.get_parameter("normalize_illumination").value):
            lab = cv2.cvtColor(corrected, cv2.COLOR_BGR2LAB)
            light, channel_a, channel_b = cv2.split(lab)
            light = cv2.createCLAHE(2.0, (8, 8)).apply(light)
            corrected = cv2.cvtColor(
                cv2.merge((light, channel_a, channel_b)), cv2.COLOR_LAB2BGR
            )

        height, width = corrected.shape[:2]
        source = self._scale_points(
            flattened_points(self.get_parameter("src_points").value),
            width,
            height,
            self.points_normalized,
        )
        destination = self._scale_points(
            flattened_points(self.get_parameter("dst_points").value),
            self.bev_width,
            self.bev_height,
            self.points_normalized,
        )
        transform = cv2.getPerspectiveTransform(source, destination)
        size = (self.bev_width, self.bev_height)
        bev = cv2.warpPerspective(corrected, transform, size, flags=cv2.INTER_LINEAR)

        roi = self._scale_points(
            flattened_points(self.get_parameter("roi_points").value),
            width,
            height,
            self.points_normalized,
        )
        source_mask = np.zeros((height, width), np.uint8)
        cv2.fillConvexPoly(source_mask, np.rint(roi).astype(np.int32), 255)
        mask = cv2.warpPerspective(
            source_mask, transform, size, flags=cv2.INTER_NEAREST
        )
        robot_rows = int(
            np.clip(float(self.get_parameter("robot_mask_fraction").value), 0.0, 0.5)
            * self.bev_height
        )
        if robot_rows:
            mask[self.bev_height - robot_rows :, :] = 0
        far_weight = float(
            np.clip(self.get_parameter("far_mask_weight").value, 0.0, 1.0)
        )
        confidence = np.linspace(
            far_weight, 1.0, self.bev_height, dtype=np.float32
        )
        mask = np.rint(mask.astype(np.float32) * confidence[:, None]).astype(
            np.uint8
        )
        bev[mask == 0] = 0

        header_frame = str(self.get_parameter("output_frame").value)
        image_message = self.bridge.cv2_to_imgmsg(bev, encoding="bgr8")
        image_message.header.stamp = stamp
        image_message.header.frame_id = header_frame
        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = image_message.header
        self.publisher.publish(image_message)
        self.mask_publisher.publish(mask_message)

    def destroy_node(self) -> bool:
        self.capture_stop.set()
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraBevNode()
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
