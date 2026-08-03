"""
로봇 연결 없이 노트북 웹캠으로 Follow-Me 로직을 미리 검증하는 스크립트
--------------------------------------------------------------------
ROS2/로봇 연결이 없어도 "카메라 -> YOLO 사람 인식 -> 좌우/거리 판단"까지의
핵심 로직을 노트북에서 바로 테스트할 수 있게 만든 스크립트입니다.

나중에 로봇이 연결되면:
  - 여기 있는 판단 로직(좌회전/우회전/직진/정지)은 이미
    robot_project/robot_project/follow/follow_person.py 에 ROS2 버전으로 옮겨져 있습니다.
  - 카메라 소스만 웹캠(cv2.VideoCapture(0)) 대신 로봇 카메라 토픽으로 바꾸면 되고
  - print() 자리를 /cmd_vel 발행으로 바꾸면 됩니다 (follow_person.py에 이미 구현됨).

사용 모델:
  - 팀원의 커스텀 YOLO(best.pt)가 아직 없어도 되도록,
    ultralytics 기본 사전학습 모델(yolov8n.pt)을 사용합니다.
  - 이 모델도 COCO 데이터셋 기준 'person' 클래스를 이미 인식하므로
    지금 로직 검증에는 충분합니다.
  - 나중에 팀원 모델(best.pt)이 오면 아래 MODEL_PATH 한 줄만 바꾸면 됩니다.

설치 (윈도우 cmd / PowerShell에서, 노트북에 Python 3.9+ 필요):
    pip install ultralytics opencv-python

실행:
    python webcam_follow_test.py
    (종료: 영상 창에서 q 키. 첫 실행 시 yolov8n.pt 자동 다운로드됨)
"""

import cv2
from ultralytics import YOLO

# ---- 설정값 (follow_person.py와 동일한 기준값 사용) ----
MODEL_PATH = 'yolov8n.pt'          # 나중에 팀원 모델로 교체: 예) 'best.pt'
CENTER_TOLERANCE_RATIO = 0.15      # 이 비율 안쪽이면 '가운데'로 간주 (follow_person.py와 동일)
STOP_HEIGHT_RATIO = 0.6            # 박스 높이가 프레임의 이 비율 이상이면 '가까움' -> 정지
PERSON_CLASS_ID = 0                # COCO 데이터셋 기준 'person' 클래스 번호

# [ADDED] 화면(cv2.putText)에는 영어로 표시 - OpenCV 기본 폰트가 한글(유니코드)을 지원하지 않아서
# 화면에 한글을 그리면 물음표(?)로 깨져 보이는 문제가 있음. 터미널 print()는 한글 그대로 유지.
TURN_LABEL_EN = {
    '좌회전': 'LEFT',
    '우회전': 'RIGHT',
    '직진(회전없음)': 'CENTER',
}
MOVE_LABEL_EN = {
    '정지(가까움)': 'STOP',
    '전진': 'FORWARD',
}


def pick_largest_person(results):
    """탐지된 사람 중 가장 큰(=가장 가까운) 박스 하나를 선택"""
    best_box = None
    best_area = 0

    for box in results.boxes:
        cls_id = int(box.cls[0])
        if cls_id != PERSON_CLASS_ID:
            continue

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best_area = area
            best_box = (x1, y1, x2, y2)

    if best_box is None:
        return None

    x1, y1, x2, y2 = best_box
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2
    bw = x2 - x1
    bh = y2 - y1
    return cx, cy, bw, bh


def decide_action(cx, bh, frame_w, frame_h):
    """follow_person.py와 동일한 좌우/거리 판단 로직 (여기서 먼저 검증 후 그대로 이식)"""
    center_x = frame_w / 2
    tolerance = frame_w * CENTER_TOLERANCE_RATIO

    if cx < center_x - tolerance:
        turn = '좌회전'
    elif cx > center_x + tolerance:
        turn = '우회전'
    else:
        turn = '직진(회전없음)'

    height_ratio = bh / frame_h
    move = '정지(가까움)' if height_ratio >= STOP_HEIGHT_RATIO else '전진'

    return turn, move


def main():
    print('YOLO 모델 로딩 중... (처음 실행이면 다운로드 때문에 시간이 좀 걸릴 수 있음)')
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print('웹캠을 열 수 없습니다. 카메라 연결/권한을 확인하세요.')
        return

    print('준비 완료. 영상 창에서 q 키를 누르면 종료됩니다.')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        results = model(frame, verbose=False)[0]
        detection = pick_largest_person(results)

        if detection is not None:
            cx, cy, bw, bh = detection
            turn, move = decide_action(cx, bh, w, h)

            cv2.rectangle(
                frame,
                (int(cx - bw / 2), int(cy - bh / 2)),
                (int(cx + bw / 2), int(cy + bh / 2)),
                (0, 255, 0), 2,
            )
            label = f'{TURN_LABEL_EN[turn]} / {MOVE_LABEL_EN[move]}'
            cv2.putText(frame, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            print(f'{turn} / {move}')
        else:
            cv2.putText(frame, 'NO PERSON -> STOP', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow('Follow-Me Logic Test (no robot needed)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
