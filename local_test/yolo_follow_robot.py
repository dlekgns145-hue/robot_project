"""
yolo_follow_robot.py - YOLO Follow-Me를 실제 로봇에 전송하는 최종 스크립트
--------------------------------------------------------------------------
[2026-07-15 깜빡임 방지 추가]
  로그 분석 결과, 사람을 정지 상태에서 계속 보고 있어도 한두 프레임씩
  인식이 끊겼다 이어지는 현상(flickering)이 있었고, 그때마다 last_box가
  즉시 초기화되면서 처음부터 다시 정렬(ALIGN)을 시작해 제자리에서
  "찾음-정렬-정지-놓침"만 반복하는 것처럼 보이는 문제가 있었음.

  해결: GRACE_FRAMES 이내로 짧게 놓친 경우에는 이전 위치/서보값을
  그대로 유지해서, 짧은 인식 끊김에는 흔들리지 않도록 함.
  LOST_FRAME_LIMIT(더 긴 시간) 이상 계속 못 찾을 때만 진짜 SEARCH 모드로 전환.

[2026-07-15 이전 재설계] 팀원 피드백 반영
  - SEARCH 모드: 카메라 완전 고정, 몸체가 SEARCH_DIRECTION 한 방향으로만
    계속 회전 (좌우 번갈아 하지 않음 -> 원점 복귀 문제 해결).
  - 사람을 다시 찾는 즉시 SEARCH를 확실히 종료하고 바로 추적/전진 재개.
  - 평소 추적 중: 카메라가 부드럽게 따라가고, 몸체는 카메라 각도가
    ALIGN_THRESHOLD를 넘을 때만 정지 후 정렬 회전, 넘지 않으면 전진.

카메라 상하 각도(servo_s2)는 robot_cmd_bridge.py 시작 시 한 번만
SERVO_TILT_DEFAULT(-60)로 고정됩니다.
장애물 회피(후진 포함, 갇힘 탈출)는 robot_cmd_bridge.py에서 독립적으로 처리됨.

실행:
    python yolo_follow_robot.py
    (종료: 영상 창에서 q 키. 종료 시 자동으로 정지 명령 전송됨)
"""

import socket
import json
import cv2
from ultralytics import YOLO

# ---- 로봇 연결 설정 ----
ROBOT_IP = "172.30.1.76"
ROBOT_PORT = 9999

# ---- YOLO / 판단 기준값 ----
MODEL_PATH = 'yolov8n-pose.pt'
STOP_HEIGHT_RATIO = 0.55
PERSON_CLASS_ID = 0

UPPER_BODY_KPT_INDICES = [0, 1, 2, 3, 4, 5, 6]
KPT_CONF_THRESHOLD = 0.5

LINEAR_SPEED = 0.35
ANGULAR_SPEED = 0.4                 # 정지 상태 제자리 정렬 회전 속도

# ---- 카메라 서보(좌우 팬) 설정 (추적 중에만 사용) ----
SERVO_PAN_MAX_ANGLE = 60
SERVO_MAX_STEP = 6
SERVO_DEADZONE_ANGLE = 25

# ---- "정지 후 정렬 -> 전진" 설정 ----
ALIGN_THRESHOLD = 20

# ---- 사람 놓쳤을 때: 몸체 한 방향 회전 탐색 ----
LOST_FRAME_LIMIT = 10               # 이 프레임 이상 연속으로 못 찾으면 SEARCH 시작
GRACE_FRAMES = 3                    # 이 프레임 이내로 짧게 놓친 건 무시(이전 위치 유지) - 깜빡임 방지
SEARCH_BODY_ANGULAR_SPEED = 0.3     # SEARCH 시 몸체 회전 속도
SEARCH_DIRECTION = 1                # 항상 이 방향으로만 회전 (1: 왼쪽, -1: 오른쪽) - 원점복귀 방지 핵심

# ---- 성능(CPU 전용 환경) 개선 설정 ----
FRAME_SKIP = 2
YOLO_IMGSZ = 320


def pick_upper_body_target(results):
    boxes = results.boxes
    if boxes is None or len(boxes) == 0:
        return None

    best_idx = None
    best_area = 0
    for i in range(len(boxes)):
        cls_id = int(boxes.cls[i])
        if cls_id != PERSON_CLASS_ID:
            continue
        x1, y1, x2, y2 = boxes.xyxy[i].tolist()
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best_area = area
            best_idx = i

    if best_idx is None:
        return None

    full_x1, full_y1, full_x2, full_y2 = boxes.xyxy[best_idx].tolist()

    if results.keypoints is None:
        cx = (full_x1 + full_x2) / 2
        cy = (full_y1 + full_y2) / 2
        return cx, cy, full_x2 - full_x1, full_y2 - full_y1

    kpts_xy = results.keypoints.xy[best_idx]
    kpts_conf = results.keypoints.conf[best_idx]

    valid_points = []
    for idx in UPPER_BODY_KPT_INDICES:
        conf = float(kpts_conf[idx])
        if conf >= KPT_CONF_THRESHOLD:
            x, y = kpts_xy[idx].tolist()
            valid_points.append((x, y))

    if len(valid_points) < 2:
        cx = (full_x1 + full_x2) / 2
        cy = (full_y1 + full_y2) / 2
        return cx, cy, full_x2 - full_x1, full_y2 - full_y1

    xs = [p[0] for p in valid_points]
    ys = [p[1] for p in valid_points]
    x1, x2 = min(xs), max(xs)
    y1, y2 = min(ys), max(ys)

    pad_x = (x2 - x1) * 0.3 + 10
    pad_y = (y2 - y1) * 0.3 + 10
    x1 -= pad_x
    x2 += pad_x
    y1 -= pad_y
    y2 += pad_y

    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    bw = x2 - x1
    bh = y2 - y1
    return cx, cy, bw, bh


def compute_servo_target(cx, frame_w):
    center_x = frame_w / 2
    offset_ratio = (cx - center_x) / center_x
    offset_ratio = max(-1.0, min(1.0, offset_ratio))
    target = int(offset_ratio * SERVO_PAN_MAX_ANGLE)

    if -SERVO_DEADZONE_ANGLE <= target <= SERVO_DEADZONE_ANGLE:
        return 0
    return target


def decide_movement(servo_current, bh, frame_h):
    if abs(servo_current) > ALIGN_THRESHOLD:
        angular = -ANGULAR_SPEED if servo_current > 0 else ANGULAR_SPEED
        turn_label = 'RIGHT(ALIGN)' if servo_current > 0 else 'LEFT(ALIGN)'
        return 0.0, angular, turn_label

    height_ratio = bh / frame_h
    if height_ratio >= STOP_HEIGHT_RATIO:
        return 0.0, 0.0, 'STOP'
    return LINEAR_SPEED, 0.0, 'FORWARD'


def smooth_servo(current, target, max_step):
    if target is None:
        return current
    diff = target - current
    if diff > max_step:
        diff = max_step
    elif diff < -max_step:
        diff = -max_step
    return current + diff


def connect_robot():
    if ROBOT_IP is None:
        return None
    print(f'{ROBOT_IP}:{ROBOT_PORT} 연결 시도...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ROBOT_IP, ROBOT_PORT))
    print('로봇 연결 성공!')
    return sock


def send_cmd(sock, linear, angular, servo_pan=None):
    if sock is None:
        return
    payload = {"linear": linear, "angular": angular}
    if servo_pan is not None:
        payload["servo_pan"] = servo_pan
    msg = json.dumps(payload) + '\n'
    sock.sendall(msg.encode('utf-8'))


def main():
    print('YOLO Pose 모델 로딩 중...')
    model = YOLO(MODEL_PATH)
    sock = connect_robot()

    cap = cv2.VideoCapture(f'http://{ROBOT_IP}:8080/stream.mjpg')
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print('스트림을 열 수 없습니다.')
        return

    mode = '로봇 연결됨 - 실제 명령 전송' if sock else '로컬 테스트 모드'
    print(f'준비 완료 ({mode}). q 키로 종료.')

    frame_count = 0
    last_box = None
    servo_target = None
    servo_current = 0

    lost_frame_count = 0
    is_searching = False

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            h, w, _ = frame.shape
            frame_count += 1
            run_inference = (frame_count % FRAME_SKIP == 0)

            person_found_this_frame = False

            if run_inference:
                results = model(frame, verbose=False, conf=0.5, imgsz=YOLO_IMGSZ)[0]
                detection = pick_upper_body_target(results)

                if detection is not None:
                    person_found_this_frame = True
                    lost_frame_count = 0
                    cx, cy, bw, bh = detection
                    servo_target = compute_servo_target(cx, w)
                    last_box = (cx, cy, bw, bh)

                    if is_searching:
                        is_searching = False
                else:
                    lost_frame_count += 1
                    # 짧게 놓친 경우(GRACE_FRAMES 이내)는 이전 위치/서보값을 그대로 유지
                    # -> 잠깐 인식 끊겨도 처음부터 다시 정렬하지 않고 부드럽게 이어짐
                    if lost_frame_count > GRACE_FRAMES:
                        servo_target = None
                        last_box = None
                    if lost_frame_count >= LOST_FRAME_LIMIT:
                        is_searching = True

            if is_searching:
                linear = 0.0
                body_angular = SEARCH_BODY_ANGULAR_SPEED * SEARCH_DIRECTION
                turn_label = 'SEARCH(body rotate, fixed dir)'
                servo_current = 0
                servo_to_send = 0

            elif last_box is not None:
                servo_current = smooth_servo(servo_current, servo_target, SERVO_MAX_STEP)
                servo_to_send = servo_current
                cx, cy, bw, bh = last_box
                linear, body_angular, turn_label = decide_movement(servo_to_send, bh, h)

            else:
                linear, body_angular, turn_label = 0.0, 0.0, 'NO PERSON -> STOP'
                servo_to_send = None

            send_cmd(sock, linear, body_angular, servo_to_send)

            if last_box is not None:
                cx, cy, bw, bh = last_box
                cv2.rectangle(
                    frame,
                    (int(cx - bw / 2), int(cy - bh / 2)),
                    (int(cx + bw / 2), int(cy + bh / 2)),
                    (0, 255, 0), 2,
                )
            display_label = f'{turn_label} / servo={servo_to_send}'
            cv2.putText(frame, display_label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('YOLO Follow Robot', frame)

            if run_inference:
                print(f'linear={linear:.2f} angular={body_angular:.2f} servo={servo_to_send}  '
                      f'({display_label}) [found_this_frame={person_found_this_frame}]')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        send_cmd(sock, 0.0, 0.0)
        if sock:
            sock.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()