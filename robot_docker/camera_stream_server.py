#!/usr/bin/env python3
"""
camera_stream_server.py - 로봇(Pi) 위에서 실행하는 파일
------------------------------------------------------------------
로봇에 달린 USB 카메라(/dev/video0) 영상을 캡처해서,
네트워크로 MJPEG 스트리밍하는 아주 가벼운 서버입니다.

노트북은 이 주소를 웹캠 대신 사용하면 됩니다:
    http://<로봇IP>:8080/stream.mjpg

예) http://172.30.1.76:8080/stream.mjpg

실행 (로봇 Docker 컨테이너 안 또는 Pi 자체에서, opencv 설치되어 있어야 함):
    python3 camera_stream_server.py
    -> "camera stream ready on 0.0.0.0:8080" 뜨면 정상

주의:
    - 라즈베리파이 연산이 약하므로 여기서는 절대 YOLO를 돌리지 않습니다.
      순수하게 "카메라 영상을 그대로 흘려보내는" 역할만 합니다.
      YOLO(follow-me용 person-detect)는 같은 로봇 안의 다른 컨테이너에서
      /camera/image_raw ROS 토픽을 구독해서 처리합니다 (아래 참고).

    - /dev/video0는 한 프로세스만 열 수 있어서, 캡처는 여기서 한 번만 하고
      그 프레임을 MJPEG(HTTP)로도 보내고 ROS 토픽(/camera/image_raw)으로도
      같이 발행합니다 -- 별도 노드를 또 만들면 장치를 못 엽니다.
    - 해상도/화질을 낮추면 네트워크 지연이 줄어듭니다. 필요시 아래
      FRAME_WIDTH / FRAME_HEIGHT / JPEG_QUALITY 값을 조정하세요.
"""

import cv2
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_INDEX = 0  # /dev/video0
FRAME_WIDTH = 320  # 640
FRAME_HEIGHT = 240  # 480
JPEG_QUALITY = 50  # 70         # 0~100, 낮을수록 빠르고 화질은 떨어짐
TARGET_FPS = 15
HOST = "0.0.0.0"
PORT = 8080

latest_frame = None
frame_lock = threading.Lock()
stop_flag = False

ros_node = None
image_publisher = None
cv_bridge = CvBridge()


def capture_loop():
    global latest_frame, stop_flag
    interval = 1.0 / TARGET_FPS
    # A USB glitch/replug can make cap.read() fail forever on an already-open
    # handle even though the device comes back. Reopen VideoCapture on any
    # read failure instead of retrying in place -- the same pattern already
    # used by map_texture_recorder.py's and camera_obstacle_guard.py's own
    # capture loops, which this one had drifted from.
    while not stop_flag:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

        if not cap.isOpened():
            print(f"[camera_stream] ERROR: /dev/video{CAMERA_INDEX} 를 열 수 없습니다. 재시도...")
            cap.release()
            time.sleep(1.0)
            continue

        print(
            f"[camera_stream] 카메라 열림 (index={CAMERA_INDEX}, "
            f"{FRAME_WIDTH}x{FRAME_HEIGHT} @ {TARGET_FPS}fps 목표)"
        )

        while not stop_flag:
            ok, frame = cap.read()
            if not ok:
                print("[camera_stream] 프레임 읽기 실패 - 카메라 재연결")
                break

            ok, jpeg = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if ok:
                with frame_lock:
                    latest_frame = jpeg.tobytes()

            if image_publisher is not None:
                try:
                    image_msg = cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    image_msg.header.stamp = ros_node.get_clock().now().to_msg()
                    image_msg.header.frame_id = "camera"
                    image_publisher.publish(image_msg)
                except Exception as error:
                    print(f"[camera_stream] ROS 이미지 발행 실패: {error}")

            time.sleep(interval)

        cap.release()
        if not stop_flag:
            time.sleep(0.5)


class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # 콘솔 로그 스팸 방지

    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not found. Use /stream.mjpg")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        try:
            while not stop_flag:
                with frame_lock:
                    frame = latest_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / TARGET_FPS)
        except (BrokenPipeError, ConnectionResetError):
            print("[camera_stream] 클라이언트 연결 종료")


def main():
    global stop_flag, ros_node, image_publisher

    rclpy.init()
    ros_node = Node("camera_stream_node")
    image_publisher = ros_node.create_publisher(Image, "/camera/image_raw", 10)

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    time.sleep(1.0)
    if stop_flag:
        print("[camera_stream] 카메라 초기화 실패로 서버를 시작하지 않습니다.")
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    server = ThreadingHTTPServer((HOST, PORT), StreamHandler)
    print(f"camera stream ready on {HOST}:{PORT}")
    print(f"  -> 노트북에서 사용할 주소: http://<로봇IP>:{PORT}/stream.mjpg")
    print("  -> ROS 토픽: /camera/image_raw")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag = True
        server.shutdown()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
