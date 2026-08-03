#!/usr/bin/env python3
"""
YOLO 기반 사람 인식 노드 (detect.py) - 확정 버전
--------------------------------------------------
카메라 이미지를 받아 YOLO 모델로 사람을 탐지하고,
탐지 결과(중심좌표, 박스 크기)를 /person_detection 토픽으로 발행한다.

발행 메시지 포맷 (follow_person.py와 호환 - 변경 없음):
    [found, center_x, center_y, box_width, box_height, frame_width, frame_height]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloDetectNode(Node):
    def __init__(self):
        super().__init__('yolo_detect_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'perception/best.pt')
        self.declare_parameter('show_debug_window', True)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value

        self.bridge = CvBridge()

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO 모델 로드 완료 (model_path={model_path})')

        self.sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/person_detection', 10)

        self.get_logger().info(f'YOLO Detect Node 시작됨 (camera_topic={camera_topic})')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        found, cx, cy, bw, bh = self.run_inference(frame)

        result = Float32MultiArray()
        result.data = [float(found), float(cx), float(cy), float(bw), float(bh), float(w), float(h)]
        self.pub.publish(result)

        if self.show_debug:
            if found:
                cv2.rectangle(
                    frame,
                    (int(cx - bw / 2), int(cy - bh / 2)),
                    (int(cx + bw / 2), int(cy + bh / 2)),
                    (0, 255, 0), 2,
                )
            cv2.imshow('YOLO Detect Debug', frame)
            cv2.waitKey(1)

    def run_inference(self, frame):
        results = self.model(frame, verbose=False)[0]
        best_box = None
        best_area = 0
        for box in results.boxes:
            cls = int(box.cls[0])
            if cls != 0:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            area = (x2 - x1) * (y2 - y1)
            if area > best_area:
                best_area = area
                best_box = (x1, y1, x2, y2)
        if best_box is None:
            return 0, 0, 0, 0, 0
        x1, y1, x2, y2 = best_box
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        bw = x2 - x1
        bh = y2 - y1
        return 1, cx, cy, bw, bh


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

