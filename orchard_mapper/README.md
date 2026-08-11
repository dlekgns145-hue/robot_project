# Orchard Mapper

2D LiDAR SLAM 지도와 같은 `map` 좌표계에 RGB 카메라 바닥 영상을 누적하는
ROS 2 Python 패키지다. LiDAR 지도는 주행 안전과 위치 추정에 사용하고, 이
패키지의 PNG는 사람이 보는 실사형 지도로 분리해 저장한다.

## 1. 전체 시스템 구조

```text
Raspberry Pi / robot                    Ubuntu compute server
┌──────────────────────┐               ┌────────────────────────────┐
│ 2D LiDAR -> /scan    │── ROS 2 ─────>│ slam_toolbox -> /map + TF  │
│ RGB -> ROS Image 또는│── ROS/MJPEG ─>│ camera_bev                 │
│       MJPEG stream   │               │   undistort -> ROI -> BEV  │
│ ESP32 -> odom/motor  │── ROS 2 ─────>│ orchard_visual_mapper      │
└──────────────────────┘               │   TF + weighted mosaic     │
                                       │   SQLite frame DB + PNG    │
                                       └────────────────────────────┘
```

Pi는 센서 I/O와 영상 인코딩에 집중하고, BEV/블렌딩/프레임 저장은 Ubuntu
서버에서 한다. 로봇과 서버 연결이 끊겨도 모터 정지 안전 로직은 Pi/ESP32 쪽에
남아 있어야 한다.

## 2. 좌표계

필수 TF 트리는 `map -> odom -> base_link`다.

- `camera`: 렌즈의 3차원 좌표. 고정 설치 변환은 `base_link -> camera`로 둔다.
- `base_link`: ROS 평면 기준으로 `+x` 전진, `+y` 왼쪽, `+z` 위쪽이다.
- `odom`: 짧은 구간에서 연속적인 휠 오도메트리 좌표다.
- `map`: SLAM loop closure로 `map -> odom`이 보정될 수 있는 전역 좌표다.
- 원본 이미지 `(u,v)`: 오른쪽으로 `+u`, 아래로 `+v`다.
- BEV 이미지: 아래 중앙이 로봇, 위쪽이 로봇 전진 방향이다.
- 전역 PNG: 북쪽(`map +y`)이 위다. 따라서 이미지 행은 map y와 반대다.

`map`의 `(x,y,yaw)`는 직접 적분하지 않고 영상 timestamp 시점의 TF
`map -> base_link`에서 얻는다. Quaternion yaw는 다음과 같다.

```text
yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
```

## 3. Camera -> BEV

ROS 입력이면 `/camera/image_raw`, `/camera/camera_info`를 사용한다. 현재 로봇
같이 MJPEG만 있으면 `input_mode: mjpeg`와 `camera_url`을 사용한다. `CameraInfo`
또는 YAML의 `camera_matrix`, `distortion_coefficients`로 `cv2.undistort`를 먼저
수행한다. 보정 전에는 ROS 2 Humble의 빈 배열 타입 문제를 피하기 위해 두 값을
`[0.0]` sentinel로 유지한다.

`src_points`는 원본 이미지에서 실제 바닥인 사다리꼴의 네 점이고,
`dst_points`는 BEV 직사각형의 대응점이다. 네 점으로
`cv2.getPerspectiveTransform`을 만들고 `cv2.warpPerspective`로 투영한다.
점은 기본적으로 `[0,1]` 정규화 좌표이므로 카메라 해상도가 바뀌어도 비율이
유지된다. 이것은 평면 바닥 가정의 inverse perspective mapping이며 나무줄기나
수관 같은 수직 물체를 진짜 항공사진처럼 복원하는 3D 방식은 아니다.

## 4. BEV -> Robot Coordinate

BEV 크기를 `W x H`, 전방 범위를 `F`, 좌우 각각의 범위를 `S`라고 하면:

```text
x_robot = (H - 1 - v) / (H - 1) * F
y_robot = (0.5 - u / (W - 1)) * 2S
```

따라서 아래 중앙은 `(0,0)`, 위 중앙은 `(F,0)`, 왼쪽은 `+y`, 오른쪽은
`-y`가 된다. YAML의 BEV 크기와 거리 범위는 두 노드에서 반드시 같아야 한다.

## 5. Robot Coordinate -> SLAM Map

TF에서 얻은 로봇 pose를 `(r_x,r_y,theta)`라고 하면:

```text
x_map = r_x + cos(theta) * x_robot - sin(theta) * y_robot
y_map = r_y + sin(theta) * x_robot + cos(theta) * y_robot
```

전역 캔버스 해상도를 `R`, 남서쪽 원점을 `(o_x,o_y)`, 이미지 높이를 `M_H`
라고 하면:

```text
u_global = (x_map - o_x) / R
v_global = M_H - 1 - (y_map - o_y) / R
```

이 연쇄 변환은 `coordinate_transform.py`에 독립 함수로 구현되어 테스트할 수
있다. OccupancyGrid가 들어오면 기본적으로 그 지도의 월드 원점과 범위를 따라
캔버스를 재배치하므로 두 지도는 같은 `map` 미터 좌표를 갖는다.

## 6. Global Image Stitching

각 BEV 네 꼭짓점을 전역 픽셀로 변환해 homography로 canvas에 warp한다. 유효
바닥 mask 안쪽에는 distance transform 기반 feather weight를 주며 다음 두
float32 배열을 누적한다.

```text
global_sum    += new_image * new_weight
global_weight += new_weight
result         = global_sum / max(global_weight, epsilon)
```

따라서 단순 덮어쓰기보다 경계가 부드럽고 `*_weight.npy`에는 각 위치의 누적
관측 강도가 남는다. 이동 0.1 m 및 회전 5도 미만이면 프레임을 건너뛴다.

## 7. ROS 2 Node 구조

- `camera_bev_node`: ROS Image 또는 MJPEG 취득, 왜곡 보정, ROI mask, BEV 발행.
- `global_visual_mapper`: BEV/mask 동기화, timestamp TF 조회, 전역 배치,
  SQLite 원본 프레임 저장, 실시간 visual map 발행.
- `visual_map_saver`: `/orchard_visual_mapper/save_now` 수동 저장 진입점.

주요 인터페이스:

```text
/camera_bev/bev/image              sensor_msgs/Image
/camera_bev/bev/mask               sensor_msgs/Image
/orchard_visual_map/image          sensor_msgs/Image
/orchard_visual_mapper/save        std_srvs/Trigger
/orchard_visual_mapper/rerender    std_srvs/Trigger
/orchard_visual_mapper/reset       std_srvs/Trigger
```

`reset`은 canvas만 지우고 프레임 DB는 보존한다.

## 8. 프로젝트 폴더

```text
orchard_mapper/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/orchard_mapper
├── orchard_mapper/
│   ├── camera_bev_node.py
│   ├── global_visual_mapper.py
│   ├── coordinate_transform.py
│   ├── image_blender.py
│   ├── frame_database.py
│   └── map_saver.py
├── config/mapper.yaml
├── launch/orchard_mapper.launch.py
└── test/
```

## 9. 전체 코드 위치

실행 가능한 전체 코드는 위 폴더의 각 파일에 들어 있다. 좌표/블렌더는 ROS에
의존하지 않기 때문에 개발 컴퓨터에서도 단위 테스트할 수 있다. `mapper.yaml`이
카메라, BEV, 지도, 이동 threshold, 저장 경로의 단일 설정 지점이다.

## 10. 설치 패키지

ROS 2 Humble Ubuntu 22.04 기준:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-message-filters \
  ros-humble-tf2-ros python3-opencv python3-numpy
```

Ubuntu 26.04에는 ROS 2 Humble 바이너리 패키지가 공식 대상이 아니므로 이
프로젝트의 Docker Ubuntu 22.04/ROS Humble 컨테이너에서 실행한다.

## 11. colcon build

패키지를 ROS workspace의 `src/orchard_mapper`에 놓은 뒤:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select orchard_mapper
source install/setup.bash
```

## 12. 실행

SLAM과 `map -> odom -> base_link` TF가 먼저 존재해야 한다.

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/path/to/slam_params.yaml

ros2 launch orchard_mapper orchard_mapper.launch.py \
  params_file:=/path/to/mapper.yaml tf_topic:=/tf
```

현재 서버처럼 필터된 TF가 `/tf_nav`에 있으면 `tf_topic:=/tf_nav`로 실행한다.
MJPEG는 촬영 timestamp가 없으므로 `allow_latest_tf_fallback: true`를 사용한다.
하드웨어 timestamp가 있는 ROS Image 입력은 이 값을 `false`로 바꿔 엄격히
동기화할 수 있다.

```bash
ros2 service call /orchard_visual_mapper/save std_srvs/srv/Trigger '{}'
ros2 service call /orchard_visual_mapper/rerender std_srvs/srv/Trigger '{}'
```

저장 결과는 `orchard_visual_map.png`, `.yaml`, `_weight.npy`,
`_metadata.json`이며 LiDAR PGM/YAML과 별개다.

## 13. Camera BEV Calibration

1. 카메라를 실제 장착 높이와 각도로 단단히 고정한다.
2. ROS 카메라라면 checkerboard와 `camera_calibration`으로 K/D를 구하고
   `/camera/camera_info`가 발행되는지 확인한다.
3. 바닥에 전방 거리와 좌우 거리를 잰 직사각형 표식을 놓는다.
4. 원본 영상에서 그 직사각형 네 모서리를 `src_points`에 시계 방향으로 넣는다.
5. `dst_points`는 전체 BEV 사각형으로 두고 `bev_forward_range`와
   `bev_side_range`를 실측값에 맞춘다.
6. BEV에서 1 m 정사각형이 평행하고 같은 크기로 보이는지 확인한다.
7. 차체가 보이면 `robot_mask_fraction`, 왜곡된 먼 영역은 `roi_points`를 줄인다.

스마트폰 자동 초점/손떨림 보정/디지털 줌은 내부 파라미터를 바꾸므로 고정할
수 있어야 한다. 손으로 든 스마트폰 영상은 로봇 pose와 외부 파라미터가 없어
정량 정합 테스트에는 쓸 수 없고, MJPEG 취득과 BEV 모양 확인까지만 가능하다.

## 14. 실제 과수원 테스트

1. 바퀴를 띄운 상태에서 `/scan`, `/odom`, TF, 카메라 timestamp를 검사한다.
2. 2 x 2 m 평탄 구역을 저속 직선 주행하고 LiDAR 벽과 visual 경계 오차를 잰다.
3. 정사각 경로 후 출발점 중첩 ghosting을 확인한다.
4. 한 줄 통로 왕복, 두 줄 통로 loop 순서로 범위를 늘린다.
5. 급회전 대신 멈춤-회전-직진으로 진동과 rolling-shutter 영향을 구분한다.
6. 그늘/직사광 각각 저장하고 `normalize_illumination` 전후를 비교한다.
7. mapping 종료 후 SLAM을 저장하고 visual `rerender`, `save`를 호출한다.
8. 알려진 나무 간격을 두 PNG/YAML의 같은 map 좌표에서 비교한다.

## 15. 예상 문제와 해결

- **반복 통로/오검출 loop closure**: slam_toolbox scan matching 범위와 loop
  threshold를 보수적으로 하고, 고유한 통로 끝/표지판을 loop anchor로 쓴다.
  IMU와 양질의 휠 odometry를 추가하면 yaw drift를 크게 줄일 수 있다.
- **울퉁불퉁한 바닥**: 고정 homography는 pitch/roll 변화에 약하다. 카메라
  방진 마운트, 짧은 노출, IMU 기반 프레임 선택/동적 homography를 적용한다.
- **햇빛과 그림자**: 카메라 exposure/white balance를 고정하고 필요할 때만
  CLAHE를 켠다. 너무 어둡거나 포화된 frame의 품질 필터를 추가한다.
- **나무/수직 장애물 왜곡**: 2D LiDAR는 높이를 모르며 단안 BEV도 지면만
  정확하다. LiDAR hit 주변의 카메라 texture/semantic label을 별도 obstacle
  layer로 합성하는 것이 안전하다. 실측 3D 수관 지도는 stereo/depth/3D LiDAR가
  필요하다.
- **진동/ghosting**: `min_translation`, feather, sharpness/optical-flow 기반
  frame rejection을 조정한다. 같은 자리를 오래 촬영하지 않는다.
- **loop closure 뒤 visual 불일치**: 모든 BEV PNG, mask, timestamp,
  당시 `map` pose와 `odom` pose를 SQLite에 보관한다. mapping 후
  `/rerender`는 최종 `map -> odom`으로 재투영한다. 이것은 전역 rigid correction을
  반영한다. Pose graph가 과거 각 keyframe을 비강체적으로 다르게 고쳤다면 최종
  보정 pose를 다음 JSON 배열로 export하고 `corrected_trajectory_path`에 둬야 한다.

```json
[
  {"stamp_ns": 1234567890000000000, "x": 1.2, "y": -0.4, "yaw": 0.1},
  {"stamp_ns": 1234567890300000000, "x": 1.3, "y": -0.4, "yaw": 0.1}
]
```

재렌더러는 timestamp가 가장 가까운 최종 pose를 사용한다. 장기적으로는 rosbag에
카메라/scan/odom/TF를 모두 기록한 후, 최종 SLAM trajectory와 정확히 timestamp
동기화해 offline rendering하는 방식이 가장 재현성이 높다.

## 안전 경계

Visual Map은 사람이 보는 보조 레이어이며 costmap 장애물 판단에 직접 사용하지
않는다. 통신 지연 또는 서버 중단 시 ESP32/Pi가 즉시 정지하도록 watchdog을
별도로 유지한다.
