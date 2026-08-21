#!/usr/bin/env python3
"""Run the trained apple defect detector on the robot camera ROS topic."""

import json
import os
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


EXPECTED_CLASSES = {"apple", "damaged_apple"}


def _environment_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _environment_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


class AppleDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("apple_detection_node")
        self.model_path = os.environ.get(
            "APPLE_MODEL_PATH",
            "/opt/robot-control/apple/apple_defect_v5_local.pt",
        )
        self.camera_topic = os.environ.get("APPLE_CAMERA_TOPIC", "/camera/image_raw")
        self.confidence = max(0.01, min(1.0, _environment_float("APPLE_CONFIDENCE", 0.65)))
        self.iou = max(0.01, min(1.0, _environment_float("APPLE_IOU", 0.45)))
        self.image_size = max(160, _environment_int("APPLE_IMGSZ", 320))
        self.minimum_interval = max(
            0.1, _environment_float("APPLE_MIN_INTERVAL_SEC", 0.8)
        )
        self.bridge = CvBridge()
        self.last_inference_at = 0.0
        self.last_result_at = 0.0
        self.last_signature = None

        status_qos = QoSProfile(depth=1)
        status_qos.history = HistoryPolicy.KEEP_LAST
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/apple_detection/status", status_qos
        )

        self.status = {
            "connected": False,
            "model_ready": False,
            "source": "robot_camera",
            "state": "starting",
            "healthy_count": 0,
            "damaged_count": 0,
            "total_count": 0,
            "best_confidence": None,
            "inference_ms": None,
            "age_s": None,
            "last_error": None,
            "boxes": [],
        }
        self._publish_status()

        if not os.path.isfile(self.model_path):
            self.status.update(state="error", last_error=f"model not found: {self.model_path}")
            self._publish_status()
            raise FileNotFoundError(self.status["last_error"])

        self.get_logger().info(f"사과 분류 모델 로딩: {self.model_path}")
        self.model = YOLO(self.model_path)
        model_names = set(self.model.names.values())
        if not EXPECTED_CLASSES.issubset(model_names):
            raise RuntimeError(
                f"unexpected model classes: {sorted(model_names)}; "
                f"expected {sorted(EXPECTED_CLASSES)}"
            )
        self.status.update(model_ready=True, state="waiting_for_camera")
        self._publish_status()

        image_qos = QoSProfile(depth=1)
        image_qos.history = HistoryPolicy.KEEP_LAST
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, self.camera_topic, self._on_image, image_qos)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"사과 분류 시작: topic={self.camera_topic}, imgsz={self.image_size}, "
            f"conf={self.confidence:.2f}, interval={self.minimum_interval:.2f}s"
        )

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        self.status["connected"] = True
        if now - self.last_inference_at < self.minimum_interval:
            return
        self.last_inference_at = now

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            started_at = time.monotonic()
            prediction = self.model.predict(
                source=frame,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                agnostic_nms=True,
                verbose=False,
            )[0]
            inference_ms = round((time.monotonic() - started_at) * 1000.0, 1)
            boxes = []
            healthy_count = 0
            damaged_count = 0
            best_confidence = None

            for detection in prediction.boxes:
                class_id = int(detection.cls[0].item())
                label = str(self.model.names[class_id])
                if label not in EXPECTED_CLASSES:
                    continue
                confidence = round(float(detection.conf[0].item()), 4)
                x1, y1, x2, y2 = (
                    int(round(value)) for value in detection.xyxy[0].tolist()
                )
                boxes.append(
                    {
                        "label": label,
                        "confidence": confidence,
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                    }
                )
                if label == "damaged_apple":
                    damaged_count += 1
                else:
                    healthy_count += 1
                best_confidence = (
                    confidence
                    if best_confidence is None
                    else max(best_confidence, confidence)
                )

            state = "damaged" if damaged_count else "healthy" if healthy_count else "none"
            self.last_result_at = time.monotonic()
            self.status.update(
                model_ready=True,
                state=state,
                healthy_count=healthy_count,
                damaged_count=damaged_count,
                total_count=healthy_count + damaged_count,
                best_confidence=best_confidence,
                inference_ms=inference_ms,
                age_s=0.0,
                last_error=None,
                boxes=boxes[:20],
            )
            self._publish_status()

            signature = (state, healthy_count, damaged_count)
            if signature != self.last_signature:
                self.get_logger().info(
                    f"사과 판정={state}, 정상={healthy_count}, 손상={damaged_count}, "
                    f"추론={inference_ms:.1f}ms"
                )
                self.last_signature = signature
        except Exception as error:  # Keep the boot service alive after a bad frame.
            self.status.update(state="error", last_error=str(error), boxes=[])
            self._publish_status()
            self.get_logger().error(f"사과 분류 실패: {error}")

    def _publish_status(self) -> None:
        if self.last_result_at > 0.0:
            self.status["age_s"] = round(time.monotonic() - self.last_result_at, 2)
        message = String()
        message.data = json.dumps(self.status, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = AppleDetectionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
