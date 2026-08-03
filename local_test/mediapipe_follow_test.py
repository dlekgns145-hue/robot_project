"""
MediaPipe 기반 Follow-Me 로직 통합 테스트 (로봇 없이 웹캠으로 검증)
--------------------------------------------------------------
팀원이 만든 MediaPipe Pose 거리 추정 코드(원본 그대로 유지)에
STEP 6(좌/중앙/우 판단)을 추가해서, STEP 4~7까지 한 번에 검증하는 버전.

기존 코드에서 추가/변경된 부분만 표시:
  [ADDED] 로 주석 표시된 곳이 이번에 새로 추가한 부분입니다.

나중에 로봇 연결되면:
  - 이 좌우 판단 로직은 robot_project/robot_project/perception/detect.py 로 옮겨져서
    /person_detection 토픽으로 발행되고, follow_person.py가 이어받아 /cmd_vel을 발행합니다.

실행:
    pip install mediapipe opencv-python numpy
    python mediapipe_follow_test.py
    (종료: 영상 창에서 q 키)
"""

import cv2
import mediapipe as mp
import numpy as np

# [ADDED] follow_person.py와 통일할 좌우 판단 기준값
CENTER_TOLERANCE_RATIO = 0.15   # 프레임 폭 기준, 이 비율 안쪽이면 '가운데'
STOP_DISTANCE_M = 0.8            # 이 거리(m)보다 가까우면 정지


def start_vision_camera():
    print("[Vision] 거리별 색상 신호등 + 좌우 판단 시스템 로딩 중...")
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    cap = None
    for cam_idx in [0, 1, 2, -1]:
        cap = cv2.VideoCapture(cam_idx)
        if cap.isOpened():
            print(f"✅ [Vision] 카메라 {cam_idx}번 연결 성공!")
            break
        cap.release()

    if not cap or not cap.isOpened():
        print("\n❌ [카메라 에러] 노트북 웹캠을 감지할 수 없습니다.")
        return

    print("🎥 웹캠 색상 경보 + 좌우 판단 시스템 가동! 종료하려면 'q'를 누르세요.")

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue

        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)
        h, w, _ = frame.shape

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            x_coordinates = [lm.x for lm in landmarks]
            y_coordinates = [lm.y for lm in landmarks]

            xmin, xmax = int(min(x_coordinates) * w), int(max(x_coordinates) * w)
            ymin, ymax = int(min(y_coordinates) * h), int(max(y_coordinates) * h)

            xmin, ymin = max(0, xmin - 20), max(0, ymin - 40)
            xmax, ymax = min(w, xmax + 20), min(h, ymax + 20)

            box_height = ymax - ymin
            if box_height > 0:
                focal_modifier = 320.0
                estimated_distance = np.clip(round(focal_modifier / box_height, 2), 0.3, 4.5)
            else:
                estimated_distance = 0.0

            if 0.8 <= estimated_distance <= 1.8:
                color = (0, 255, 0)
                status_txt = "SAFE LOCK"
            elif (0.6 <= estimated_distance < 0.8) or (1.8 < estimated_distance <= 2.5):
                color = (0, 165, 255)
                status_txt = "WARNING"
            else:
                color = (0, 0, 255)
                status_txt = "DANGER"

            # [ADDED] STEP 6: 사람 중심 x좌표로 좌/중앙/우 판단
            person_center_x = (xmin + xmax) / 2
            frame_center_x = w / 2
            tolerance = w * CENTER_TOLERANCE_RATIO

            if person_center_x < frame_center_x - tolerance:
                turn_txt = "LEFT (좌회전)"
            elif person_center_x > frame_center_x + tolerance:
                turn_txt = "RIGHT (우회전)"
            else:
                turn_txt = "CENTER (직진)"

            # [ADDED] STEP 7~8: 실측 거리(m) 기준으로 정지/전진 판단
            move_txt = "STOP (정지)" if estimated_distance < STOP_DISTANCE_M else "FORWARD (전진)"

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)

            label_y = max(35, ymin)
            cv2.rectangle(frame, (xmin, label_y - 35), (xmin + 340, label_y), color, cv2.FILLED)
            text_display = f"{status_txt} | Dist: {estimated_distance} m"
            cv2.putText(frame, text_display, (xmin + 8, label_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

            # [ADDED] 좌우/이동 판단 결과를 화면 하단에 크게 표시
            action_txt = f"TURN: {turn_txt}  |  MOVE: {move_txt}"
            cv2.rectangle(frame, (0, h - 45), (w, h), (30, 30, 30), cv2.FILLED)
            cv2.putText(frame, action_txt, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            print(action_txt)

            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        else:
            cv2.rectangle(frame, (20, 15), (320, 50), (0, 0, 255), cv2.FILLED)
            cv2.putText(frame, "SEARCHING FOR TARGET...", (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            # [ADDED] 사람 없을 때도 하단에 STOP 표시
            cv2.rectangle(frame, (0, h - 45), (w, h), (30, 30, 30), cv2.FILLED)
            cv2.putText(frame, "TURN: -  |  MOVE: STOP (no target)", (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Vision AI Edge Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    start_vision_camera()
