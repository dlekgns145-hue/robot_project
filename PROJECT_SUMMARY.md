# 로봇 Follow-Me 프로젝트 정리 (팀 공유용)

작성일: 2026-07-10

## 1. 목표

Yahboom 로봇이 카메라로 사람을 인식해서, 사람을 따라 이동하게 만드는 것.
(SLAM 지도 생성, Navigation은 시간 되면 추가 - 우선순위 낮음)

## 2. 최종 구조 (지금 실제로 쓰는 방식)

라즈베리파이는 연산이 약해서 **AI 인식은 노트북에서, 로봇은 명령만 받아서 움직이는 구조**로 결정했습니다.

```
[노트북]                                    [로봇 - 라즈베리파이 / Docker]
 웹캠 촬영
   ↓
 YOLO로 사람 인식
   ↓
 좌우/거리 판단 (좌회전·우회전·직진·전진·정지)
   ↓
 TCP 소켓 전송 (포트 9999) ─────────────→  robot_cmd_bridge.py 가 수신
                                              ↓
                                          /cmd_vel 로 발행 (rclpy)
                                              ↓
                                          LiDAR(/scan)로 정면 장애물 있으면
                                          전진 명령 무시 (장애물 회피)
                                              ↓
                                          base_node_X3 → 실제 바퀴 구동
```

**중요**: 로봇 자체 카메라나 로봇 안의 YOLO는 안 씁니다. 노트북 웹캠으로 사람을 보고, 그 판단 결과(속도/회전 값)만 무선으로 로봇에 전송하는 방식입니다.

## 3. 파일별 설명

### 📁 `local_test/` - 노트북에서 실행하는 파일들 (실제로 쓰는 것)

| 파일 | 실행 위치 | 역할 |
|---|---|---|
| `yolo_follow_robot.py` | 노트북 | **최종 실행 파일.** 웹캠 → YOLO 인식 → 판단 → 로봇에 소켓 전송까지 한 번에 처리 |
| `robot_cmd_bridge.py` | 로봇 (Docker 컨테이너 안) | 소켓으로 받은 명령을 `/cmd_vel`로 변환해서 발행. LiDAR로 장애물 감지되면 전진 무시 |
| `robot_sender.py` | 노트북 | 통신 테스트 전용 (전진 2초 → 정지). YOLO 없이 연결만 확인할 때 사용 |
| `webcam_follow_test.py` | 노트북 | 로봇 연결 없이 YOLO 인식/판단 로직만 검증하는 테스트용 (화면에 LEFT/RIGHT/CENTER, STOP/FORWARD 표시) |
| `mediapipe_follow_test.py` | 노트북 | MediaPipe 버전 (백업용, 현재 미사용 - 최종적으로 YOLO로 확정함) |
| `requirements.txt` | - | 노트북에 설치할 패키지 목록 (`ultralytics`, `opencv-python`) |

### 📁 `robot_project/` - ROS2 패키지 (참고/백업용, 현재 데모엔 미사용)

원래는 로봇 안에서 카메라+YOLO+판단까지 전부 ROS2 노드로 처리하려고 만들었던 버전입니다. 지금은 위 2번 구조(노트북에서 처리)로 바꿔서 **현재 데모에는 안 쓰지만**, 나중에 로봇 자체 카메라로 전환하고 싶어지면 이 구조를 다시 쓸 수 있습니다.

| 파일 | 역할 |
|---|---|
| `perception/detect.py` | 카메라 이미지 구독 → YOLO 인식 → `/person_detection` 발행 |
| `follow/follow_person.py` | `/person_detection` 구독 → 좌우/거리 판단 → `/cmd_vel` 발행 |
| `navigation/nav.py` | Navigation(목표지점 자동이동) 골격 코드 - 우선순위 낮아서 최소 골격만 있음 |
| `launch/start.launch.py` | 위 노드들을 한번에 실행하는 launch 파일 |

## 4. 진행 상황 (STEP 대응표)

| STEP | 내용 | 상태 |
|---|---|---|
| 1 | 속도 조절 | ✅ 파라미터로 빼둠 (`LINEAR_SPEED`, `ANGULAR_SPEED`) - 실측 튜닝 필요 |
| 2 | SLAM 지도 생성 | ⏸ 보류 (Follow Me 먼저) |
| 3 | Navigation | ⏸ 골격만 존재, 우선순위 낮음 |
| 4 | 카메라 실행 | ✅ 노트북 웹캠 사용 |
| 5 | 사람 인식 (YOLO) | ✅ 완료 (일반 사전학습 모델 `yolov8n.pt` 사용 중, 커스텀 `best.pt`는 아직 못 찾음) |
| 6 | 사람 중심 계산 (좌/중앙/우) | ✅ 완료 |
| 7 | 거리 계산 (박스 크기 기반) | ✅ 완료 |
| 8 | 사람 따라가기 (cmd_vel 생성) | ✅ 로직 완료 - **로봇 실제 구동 테스트 중** (통신은 성공, 바퀴 미동작 원인 파악 중) |
| 9 | 장애물 회피 (LiDAR) | ✅ `robot_cmd_bridge.py`에 통합 (정면 0.35m 이내면 전진 차단) - 실기 테스트 아직 |

## 5. 현재 막힌 부분 (팀원 도움 필요하면 여기)

- **통신(노트북 ↔ 로봇)은 정상** — 연결 성공, 명령 전송/정지까지 에러 없이 완료됨
- **근데 실제 바퀴가 안 돎** — 원인 후보 확인 중:
  - 전원(충전기 연결 상태) 관련 가능성
  - `base_node_X3`를 수동 실행했더니 `/YB_Car_Node`라는 노드가 이미 자동으로 떠있던 것과 중복/충돌 가능성
  - Docker 컨테이너가 여러 개 떠 있어서 브리지랑 base_node가 서로 다른 컨테이너에 들어가 있을 가능성
- **커스텀 YOLO 모델(`best.pt`) 위치 불명** — 지금은 일반 모델(`yolov8n.pt`)로 사람 인식은 문제없이 되고 있어서, 급하면 이걸로 진행해도 됨
- **로봇 카메라 이미지 안 나오는 문제**는 별도로 확인 중 (SSH엔 화면이 없어서 `cv2.imshow`가 안 되는 게 원인일 가능성 - 파일로 저장해서 확인하는 방법 안내함). 단, 지금 구조(노트북 웹캠 사용)에서는 이 문제가 최종 데모에 영향 없음

## 6. 로봇 쪽 실행 순서 (요약)

1. 로봇 전원 ON → Wi-Fi `Micro_ros` 연결
2. 창① : `ssh` 접속 → `sh ~/start_agent_rpi5.sh` (켜둔 채 유지)
3. 창② : `ssh` 접속 → `./ros2_humble.sh` → `ros2 run yahboomcar_base_node base_node_X3` (켜둔 채 유지)
4. 창③ : `ssh` 접속 → `sh ~/start_bridge.sh` → `cmd_bridge ready. listening on 9999` 확인
5. 노트북 : `yolo_follow_robot.py`의 `ROBOT_IP`를 `"10.42.0.1"`로 바꾸고 실행
