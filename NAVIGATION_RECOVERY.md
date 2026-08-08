# Navigation 인수인계 및 복구 순서

이 문서는 실물 로봇 단독 Navigation 완주 검증을 위한 작업 순서다. GUI 연동은
이 절차에서 3회 연속 완주를 확인한 뒤 진행한다.

## 0. 재생성보다 백업이 먼저

`odom_relay.py`, `scan_time_fix.py`와 수정된 launch/parameter 파일은 현재 Git
저장소에 없다. 로봇 또는 컨테이너 안에서만 수정됐을 수 있으므로 아래 확인 전에는
`docker compose up --build`, `install.sh` 실행, 컨테이너 삭제를 하지 않는다.

```bash
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker diff robot-base-node
docker inspect --format '{{.Name}} image={{.Config.Image}} created={{.Created}}' robot-base-node
docker exec robot-base-node bash -lc \
  "find /root -type f \( -name 'odom_relay.py' -o -name 'scan_time_fix.py' \
  -o -name '*launch*.py' -o -name '*nav*params*.yaml' \) -print"
```

찾은 수정 파일과 실제 사용하는 지도(`*.yaml`, `*.pgm`)를 먼저 `docker cp`로
로봇 호스트에 복사한 뒤 이 저장소에 넣는다. `docker diff`에서 `C`로 표시된 파일은
원본 이미지와 달라진 파일이므로 특히 우선 회수한다.

## 1. 전원 및 런타임 확인

로봇을 평평한 바닥에 놓고 전원을 켠 뒤 5초 이상 움직이지 않아 IMU 초기화를
마친다. 그다음 agent와 Yahboom 컨테이너가 중복되지 않았는지 확인한다.

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
docker ps --filter ancestor=microros/micro-ros-agent:humble --format '{{.ID}} {{.Names}}'
docker ps --filter ancestor=yahboomtechnology/ros-humble:4.1.2 --format '{{.ID}} {{.Names}}'
```

micro-ROS agent는 하나만 있어야 한다. 기존 `smart_agent.sh`, `smart_ros2.sh`가
Compose 런타임과 동시에 별도 컨테이너를 만들지 않게 한다.

## 2. ROS 데이터와 TF 확인

아래 명령은 `robot-base-node`가 실행 중이고 모든 컨테이너가 host network와 같은
`ROS_DOMAIN_ID=20`을 쓴다는 기준이다.

```bash
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 node list'
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; timeout 8 ros2 topic hz /odom_raw'
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; timeout 8 ros2 topic hz /odom'
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; timeout 8 ros2 topic hz /scan'
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 run tf2_ros tf2_echo odom base_link'
```

확인 기준:

- `/odom_raw`, `/odom`, `/scan`이 멈추지 않고 갱신된다.
- odom과 scan의 header timestamp가 현재 시각과 크게 벌어지지 않는다.
- `odom -> base_footprint/base_link` TF에 NaN이 없다.
- Navigation과 AMCL 실행 후 `map -> odom` TF가 생긴다.

`/cmd_vel` 발행 주체도 확인한다.

```bash
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 topic info /cmd_vel --verbose'
```

현재 저장소는 GUI가 연결되지 않았을 때 `robot-command-bridge`가 정지값을 한 번만
발행하고 이후 topic을 양보하도록 수정했다. 이 수정본이 로봇에 배포되기 전에는
단독 Navigation 테스트 동안 데스크톱 GUI를 연결하지 말고
`robot-command-bridge`를 중지해야 한다.

```bash
docker stop robot-command-bridge
```

테스트가 끝나면 다음 명령으로 복구한다.

```bash
docker start robot-command-bridge
```

수정본을 배포할 때는 VM gateway를 먼저 갱신하고 로봇 bridge를 나중에 갱신한다.
새 gateway의 heartbeat와 새 bridge의 one-shot timeout stop이 함께 적용돼야 Nav2에
`/cmd_vel` 제어권을 안전하게 넘길 수 있다. 데스크톱 GUI의 Navigation 시작 경로도
같은 시점에 motor-command lease를 해제하도록 수정돼 있다.

## 3. Navigation 재현

### GUI에서 새 지도 만들기

Navigation 런타임과 mapping 런타임은 같은 Nav2 노드 이름을 사용하므로 동시에
실행하지 않는다. 로봇 호스트에서 아래처럼 매핑 모드로 전환하면 GUI의 `지도 주행 →
현장 지도 만들기` 버튼이 활성화된다.

```bash
cd /path/to/robot_docker
docker compose --profile navigation stop navigation-runtime
docker compose --profile mapping up -d mapping-runtime
```

GUI에서 `자동 매핑 시작` 후 필요할 때 `현재 지도 저장`을 누른다. 저장이 완료되면
`orchard_map.pgm/.yaml`과 함께 `orchard_map_texture.png`가 생성된다. texture는 카메라
영상을 지면으로 투영한 안내용 레이어이며, 장애물 판정과 목표 좌표는 항상 LiDAR PGM을
사용한다. 카메라를 읽지 못해도 LiDAR 지도 저장은 정상 동작한다.

매핑을 마친 뒤에는 다음처럼 Navigation 모드로 되돌린다.

```bash
docker compose --profile mapping stop mapping-runtime
docker compose --profile navigation up -d navigation-runtime
```

Navigation을 시작하고 모든 Nav2 lifecycle node가 active인지 확인한 뒤 initial pose를
발행한다. initial pose의 위치와 방향은 지도상의 실제 로봇 위치와 일치해야 한다.

Yahboom 공식 `navigation_dwb_launch.py`에 포함된 `StopCarNode` 원본은 실행 중 계속
0을 발행하지 않고 프로세스 종료 시 정지 명령을 한 번 발행한다. 현장 이미지의 파일이
다를 수 있으므로 영구 제거 전 다음 두 가지를 함께 확인한다.

```bash
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 node info /StopCarNode'
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 topic info /cmd_vel --verbose'
```

## 4. costmap 초기화 후 목표 재전송

먼저 실제 서비스 이름을 확인한다.

```bash
docker exec robot-base-node bash -lc \
  "source /opt/ros/humble/setup.bash; ros2 service list | grep clear_entirely"
```

기본 Nav2 이름이면 local/global costmap을 모두 초기화한다.

```bash
docker exec robot-base-node bash -lc \
  "source /opt/ros/humble/setup.bash; ros2 service call \
  /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'"
docker exec robot-base-node bash -lc \
  "source /opt/ros/humble/setup.bash; ros2 service call \
  /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'"
```

초기화 완료 응답을 확인한 다음 같은 목표를 다시 보낸다. 한 번 성공해도 출발 자세와
목표를 동일하게 유지해 최소 3회 연속 완주해야 완료로 판정한다.

## 5. 여전히 zero-length plan이면

costmap 잔상 가설을 기각하고 다음 순서로 좁힌다.

1. `planner_server`, `controller_server`, `bt_navigator` lifecycle 상태가 active인지 확인.
2. `tf2_echo map base_link`가 연속으로 유효한지 확인.
3. `/amcl_pose`가 실제 위치와 일치하고 covariance가 수렴하는지 확인.
4. 시작점과 목표점이 global costmap의 lethal/inflated/unknown cell인지 확인.
5. `planner_server` 로그에서 `start is occupied`, `goal is occupied`, TF timeout 등
   zero-length 직전의 최초 오류를 확인.
6. 지도 YAML이 가리키는 PGM 경로, resolution, origin과 실제 사용 지도가 맞는지 확인.

이 단계에서는 `Received plan with zero length` 자체보다 그 직전에 planner가 기록한
오류가 원인 판정의 핵심 증거다.

## 완료 조건

- 같은 조건에서 Navigation 3회 연속 완주.
- `/cmd_vel` 제어권 경합 없음.
- odom/scan timestamp 및 TF 정상.
- 수정된 relay/filter/launch/parameter/map 파일이 Git에 저장됨.
- 컨테이너 재생성 뒤에도 동일 테스트 통과.
