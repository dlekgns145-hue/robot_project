# robot_project

Yahboom ROS2 로봇 - SLAM / 카메라(OpenCV) / YOLO 사람 인식 / Follow Me / Navigation 통합 프로젝트

## 폴더 구조

```
robot_project/
├── robot_project/            # 파이썬 ROS2 노드 코드 (실제 패키지 모듈)
│   ├── perception/
│   │   └── detect.py         # STEP 4~6: 카메라 + YOLO 사람 인식 (확정)
│   ├── follow/
│   │   └── follow_person.py  # STEP 6~8: 사람 따라가기 (cmd_vel 발행)
│   └── navigation/
│       └── nav.py            # STEP 3: Navigation (낮은 우선순위, 골격만)
├── launch/
│   └── start.launch.py       # 전체 실행용 launch 파일
├── slam/
│   ├── map.yaml               # STEP 2 완료 후 생성됨
│   └── map.pgm                # STEP 2 완료 후 생성됨
├── robot_docker/             # Raspberry Pi 고정 이름 Docker 런타임
│   ├── robot_cmd_bridge.py       # TCP 명령·LiDAR 안전제어
│   └── camera_stream_server.py   # 로봇 카메라 MJPEG 서버
├── ubuntu_v2/               # 데스크톱 GUI·VM gateway·테스트
├── resource/robot_project
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 토픽 흐름 (지금 구현된 부분)

```
/camera/image_raw  →  [detect.py]  →  /person_detection  →  [follow_person.py]  →  /cmd_vel
                       (YOLO 추론)       (Float32MultiArray)      (좌우/거리 판단)
```

`/person_detection` 메시지 형식 (`std_msgs/Float32MultiArray.data`):

```
[found, center_x, center_y, box_width, box_height, frame_width, frame_height]
```

## 개발 시작 전 팀원과 확인할 것

1. **카메라 토픽 이름** - 실제 카메라 노드가 발행하는 토픽이 `/camera/image_raw`가 맞는지
   (`ros2 topic list`로 확인 후 다르면 `detect.py`의 `camera_topic` 파라미터만 바꾸면 됨)
2. **best.pt 모델 파일 위치** - `perception/detect.py`의 `model_path` 파라미터 기본값이
   `'yolo/best.pt'` (상대경로)로 되어 있음. 실행하는 위치 기준 상대경로라 헷갈리기 쉬우니,
   가능하면 절대경로로 넘기거나 실행 위치를 통일할 것 (launch 파일에서 파라미터로 지정 권장)

## 로봇(Docker 컨테이너) 안에 추가로 설치해야 하는 것

YOLO(ultralytics)는 ROS2 표준 의존성이 아니라 pip 패키지라서, 로봇 쪽 Docker 컨테이너 안에서 한 번 설치해야 합니다.

```bash
pip install ultralytics opencv-python --break-system-packages
```

## 빌드 & 실행 (로봇의 ROS2 워크스페이스 안에서)

```bash
# 1) 워크스페이스의 src 폴더에 이 robot_project 폴더를 복사한 뒤
cd ~/ros2_ws
colcon build --packages-select robot_project
source install/setup.bash

# 2) 실행
ros2 launch robot_project start.launch.py

# 속도(STEP1) 바꿔서 테스트하고 싶으면
ros2 launch robot_project start.launch.py linear_speed:=0.4
```

## 통합 실행 메인

`robot_project/main.py`가 Perception, Follow Me, Navigation의 공통 실행점입니다.
빌드 후 모드만 선택해 실행합니다.

```bash
# 카메라 + YOLO 인식만
ros2 run robot_project integrated_main --mode perception

# Perception + Follow Me 통합
ros2 run robot_project integrated_main --mode follow --ros-args \
  -p model_path:=/absolute/path/to/best.pt \
  -p camera_topic:=/camera/image_raw \
  -p linear_speed:=0.3

# Nav2에 목표 좌표 전송
ros2 run robot_project integrated_main --mode navigation --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.5 -p goal_yaw:=0.0
```

launch 파일로도 같은 모드를 선택할 수 있습니다.

```bash
ros2 launch robot_project start.launch.py mode:=perception
ros2 launch robot_project start.launch.py mode:=follow linear_speed:=0.3
ros2 launch robot_project start.launch.py mode:=navigation goal_x:=1.0 goal_y:=0.5
```

Follow Me와 Navigation은 둘 다 로봇 이동을 제어하므로 동시에 실행하지
않고 모드 전환으로 사용합니다. Navigation 모드 전에는 map server, AMCL,
planner/controller를 포함한 Nav2 bringup이 먼저 실행 중이어야 합니다.

## STEP 대응표 (기획서 기준)

| STEP | 내용 | 관련 파일 | 상태 |
|---|---|---|---|
| 1 | 속도 조절 | `follow_person.py`의 `linear_speed` 파라미터 | 파라미터화 완료, 실측 필요 |
| 2 | SLAM 지도 생성 | `slam/` 폴더 (map_saver로 저장) | 코드 아님 - 실습으로 진행 |
| 3 | Navigation | `navigation/nav.py` | 골격만 (우선순위 낮음) |
| 4 | 카메라 실행 | `perception/detect.py`의 이미지 구독부 | 구현됨 |
| 5 | 사람 인식 | `perception/detect.py` (YOLO, 확정) | **완료** |
| 6 | 사람 중심 계산 | `follow_person.py` 좌/우 판단 로직 | 구현됨 |
| 7 | 거리 계산 | `perception/detect.py` (박스 높이 기반) | 구현됨 |
| 8 | 사람 따라가기 | `follow_person.py` 전체 | 구현됨 (YOLO 연결 후 테스트 필요) |
| 9 | 장애물 회피 | 미구현 - `/scan` 구독 노드 추가 필요 | 다음 작업 |
| 10 | SLAM+YOLO 통합 | 미구현 | 다음 작업 |
| 11 | 성능 개선 | - | 마지막 |

## 다음에 할 일 (우선순위 순)

1. 카메라 토픽 이름 확인 후 `detect.py` 파라미터 맞추기
2. 팀원 YOLO 코드를 `detect.py`에 연결하고 실제 카메라로 테스트
3. `follow_person.py` 파라미터(속도, 정지 거리 기준) 실측하며 튜닝
4. 시간이 남으면 SLAM(STEP2) → 장애물 회피(STEP9) 순으로 추가
