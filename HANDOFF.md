# robot_project 인수인계 문서

**작성일:** 2026-08-13 · **작성 계기:** 2026-08-11~13 매핑 기능 실기 디버깅 세션 종료 시점
**대상 독자:** 이 프로젝트를 처음 보는 AI 에이전트 / 개발자. 아래 내용만 읽으면 코드를 처음부터 다시 뒤지지 않고 이어서 작업할 수 있게 쓴 문서.

---

## 1. 이게 뭐 하는 프로젝트인가

Yahboom **MicroROS-Pi5** 로봇(4륜, LiDAR + 카메라 짐벌)을 ROS2(Humble)로 제어하는 프로젝트.
목표는 크게 세 갈래:

1. **SLAM 자율 매핑** — LiDAR 기반으로 과수원/실내 공간 지도를 자율 탐색으로 만든다 (현재 메인 작업, 이 문서의 핵심 주제).
2. **Navigation** — 만든 지도 위에서 Nav2로 목표 지점까지 자율주행.
3. **YOLO 사람 인식 + Follow Me** — 카메라로 사람을 인식해 따라가기 (이건 이미 완료된 별도 기능, 최근 세션은 안 건드림).

지도 생성은 **LiDAR 전용**이다. 카메라 이미지/텍스처 레이어는 지도 생성·저장·전송에 절대 관여하지 않는다 — 이건 설계 원칙이고, 과거에 카메라 텍스처를 지도에 섞으려던 시도(`map_texture_core.py`, `map_texture_recorder.py`, `calibrate_map_texture.py`, `obstacle_texture_fusion.py`)가 있었지만 지금은 폐기됐다. 이 파일들은 저장소에 아직 남아있지만 **매핑 파이프라인 어디에도 연결돼 있으면 안 된다** — 연결돼 있으면 그 자체가 버그다 (실제로 오늘 이런 회귀가 있었다, 6절 참고).

---

## 2. 하드웨어

- **로봇:** Yahboom MicroROS-Pi5 (제품 페이지: yahboom.net). 4륜, "310 encoder motor" x4, 450±10rpm.
- **컴퓨트:** Raspberry Pi 5 + ESP32 코프로세서(모터 드라이버·IMU·엔코더 담당, 시리얼로 Pi5와 통신).
- **센서:** MS200 LiDAR, 2MP 카메라(2DOF 짐벌).
- **바퀴/치수:** 벤더가 휠 직경·차체 치수·무게를 공식 공개하지 않음. 게다가 **뒷바퀴가 순정보다 큰 걸로 교체됨**(2026-08-11경) — 오도메트리 스케일이 순정 스펙과 안 맞을 수 있으니 유의. 실측이 유일한 신뢰 가능한 값.
- **IMU 콜드부트 요구사항 (중요, 3번 이상 실제로 겪은 이슈):** 로봇 전원을 켠 직후 IMU가 자체 캘리브레이션을 완료하는 데 시간이 걸린다. **도커 컨테이너를 재시작하는 것만으로는 이 캘리브레이션이 재실행되지 않는다** — IMU는 ESP32에 물려 있고, ESP32는 Pi5의 docker 이벤트와 무관하게 독립적으로 동작하기 때문. 증상과 해결법은 6절 참고.

---

## 3. 저장소 구조 (실제로 만지는 부분만)

```
robot_project/
├── robot_docker/          # ★ 로봇(Pi5) 위에서 도는 도커 스택의 소스. 이게 진짜 핵심.
│   ├── compose.yaml            # 로봇 단독 모드 전체 서비스 정의
│   ├── Dockerfile               # 위 서비스들이 공유하는 단일 이미지 (오늘 여러 버그의 근원지)
│   ├── robot_cmd_bridge.py      # 최종 안전 권한 + TCP :9999 서버 + 저장된 지도 파일 서빙
│   ├── mapping_runtime_launch.py    # SLAM+Nav2+자율탐색 ROS2 launch (매핑용)
│   ├── navigation_runtime_launch.py # 저장된 지도로 주행하는 launch (AMCL)
│   ├── autonomous_mapping.py    # 프론티어 탐색 메인 조정자 노드
│   ├── frontier_core.py         # 프론티어 탐색 알고리즘 (순수 함수, ROS 의존 없음)
│   ├── mapping_core.py          # 점유격자 품질분석 + 안전한 지도파일 저장 (순수 함수)
│   ├── odom_relay.py            # 벤더 EKF 우회, /odom_raw를 finite-check 후 재발행 + TF
│   ├── scan_time_fix.py         # LiDAR 타임스탬프 보정 + 자기반사 필터링
│   ├── cmd_vel_relay.py         # Nav2 내부 고정 토픽명(/cmd_vel) → /cmd_vel_server 릴레이
│   ├── fastdds_profile.xml      # FastDDS 버퍼 리사이즈 이슈 회피용 프로파일
│   ├── mapping_slam_params.yaml # slam_toolbox 파라미터
│   ├── recovered/               # 실측으로 튜닝된 nav 파라미터(dwb_nav_params_fixed.yaml 등), 지도 백업
│   └── map_texture_*.py, calibrate_map_texture.py, obstacle_texture_fusion.py
│                                 # ⚠️ 폐기된 카메라-텍스처 매핑 실험. Dockerfile/launch에 다시
│                                 #    연결되면 안 됨 (README의 "지도 생성 원칙" 위반).
│
├── ubuntu_v2/              # ★ 별도 "컴퓨트 서버" 모드 + 데스크톱 GUI 소스
│   ├── compose.yaml             # 컴퓨트 서버 모드 서비스 정의 (gateway, compute-mapping, ros-transport)
│   ├── desktop_gui/main.py      # PySide6 데스크톱 GUI
│   ├── robot_app/robot_gateway.py   # GUI ↔ 로봇 중계 게이트웨이
│   ├── robot_app/map_payload.py     # 지도 파일 직렬화(base64+zlib)
│   └── tests/                   # pytest 스위트 (125개, 아래 8절 참고)
│
├── orchard_mapper/          # 카메라 기반 시각 매핑 실험 패키지 (매핑 코어와는 별개 트랙)
├── server_image/            # 컴퓨트 서버용 배포 번들 빌드 스크립트 (INSTALL_SERVER.sh 등)
├── NAVIGATION_RECOVERY.md   # 실기 Navigation 복구 절차 문서 (일부 최신, 4절 참고)
├── 전체_실행_순서.txt        # ⚠️ 구버전 실행 순서 (Follow Me 중심, raw shell script 기반).
│                             #    지금의 robot_docker/compose.yaml 방식과 안 맞음 — 참고만 하고
│                             #    그대로 따라하지 말 것 (4절 참고).
└── HANDOFF.md                # 이 문서
```

---

## 4. 배포 아키텍처 — 두 가지 모드가 코드에 공존한다

**중요:** 이 저장소는 두 가지 서로 다른 배포 방식을 코드로 갖고 있고, 문서(`전체_실행_순서.txt`)는 그중 오래된 방식만 설명한다. 헷갈리지 않으려면 이 구분을 먼저 이해해야 한다.

### 모드 A — 로봇 단독 모드 (`robot_docker/compose.yaml`) — **2026-08-13 기준 실제로 검증된 경로**

Pi5 하나에 모든 컨테이너가 `network_mode: host`, `ROS_DOMAIN_ID=20`으로 뜬다. SLAM/Nav2도 Pi5 위에서 직접 돈다.

```
ESP32 --serial 921600--> micro-ros-agent --ROS2--> base-node(/scan, /odom_raw, /imu)
                                                        │
                                    (raw, 벤더 EKF 우회) ▼
                                              mapping-runtime
                                   (odom_relay·slam_toolbox·Nav2·autonomous_mapping·cmd_vel_relay)
                                                        │ /cmd_vel_server
                                                        ▼
                                              command-bridge (안전 클램프)
                                                        │ /cmd_vel (최종)
                                                        ▼
                                              micro-ros-agent → ESP32 모터
```

- GUI는 `robot-command-bridge`의 **TCP :9999에 직접** 붙는다 (컴퓨트 서버 불필요).
- `robot-ros-transport`(Zenoh, ROUTER, `tcp/0.0.0.0:7448`)는 정의는 돼 있지만 상대가 안 붙으면 완전히 유휴 상태.
- `mapping-runtime`/`navigation-runtime`은 `profiles: [mapping]`/`[navigation]`로 게이트돼 있어서 **평소 부팅 시 자동 시작 안 함** (안전 설계 — 사람 없이 로봇이 혼자 탐색 주행 시작하는 걸 막기 위함). `docker compose --profile mapping up -d mapping-runtime`으로 수동 기동, `/autonomous_mapping/start` 서비스 콜로 탐색 시작.

### 모드 B — 컴퓨트 서버 모드 (`ubuntu_v2/compose.yaml`) — **문서상 "기본" 아키텍처지만 2026-08-13 기준 어디서도 안 떠 있음**

별도 Ubuntu 머신(VM 등)에서 `gateway` + `compute-mapping`/`compute-navigation` + `ros-transport`(Zenoh **CLIENT**, `172.30.1.10:7448`로 outbound 접속)가 뜬다. SLAM/Nav2 연산을 Pi5 밖으로 덜어내는 목적. GUI는 이 서버의 `gateway`(TCP :9999)에 붙고, gateway가 로봇의 `command-bridge`로 중계한다.

Zenoh 화이트리스트(`robot_docker/zenoh/robot-transport.json5` ↔ `ubuntu_v2/zenoh/server-transport.json5`)는 정확히 대칭:
- 로봇→서버: `/scan`, `/odom_raw`, `/cmd_bridge/emergency_stop`
- 서버→로봇: `/cmd_vel_server`, `/cmd_bridge/navigation_lease`, `/amcl_pose`, `/autonomous_mapping/status`, `/autonomous_mapping/(start|stop|save|preview)` 서비스, `/navigate_to_pose` 액션

### `전체_실행_순서.txt`는 왜 다른 얘기를 하나

그 문서는 `smart_agent.sh`/`start_bridge.sh` 같은 raw 쉘 스크립트로 로봇을 띄우는 **더 오래된 방식**(Follow Me 기능 중심, docker compose 이전 단계)을 설명한다. 지금은 로봇 쪽이 `robot_docker/compose.yaml`로 완전히 도커화됐다. **이 문서의 절차를 그대로 따라하지 말 것** — 로봇 쪽 실행은 4절의 모드 A 기준으로, 로봇 접속은 `ssh pi@<로봇IP>` → `docker compose ...`로 한다.

### 지도가 GUI에 어떻게 뜨나 (자주 헷갈리는 부분)

`command-bridge`가 `/opt/robot-control/maps/orchard_map.{pgm,yaml}`를 **디스크에서 직접 읽어서** GUI로 보낸다 (`robot_cmd_bridge.py`의 `load_map_payload()`). SLAM이 지금 돌고 있을 필요가 전혀 없다 — `mapping-runtime`이 지도를 한 번 저장해두면, 그 컨테이너를 꺼도 GUI는 계속 그 지도를 볼 수 있다. GUI는 연결되자마자 자동으로 지도를 요청한다 (`main.py`의 `on_status()` → `request_map()`).

---

## 5. 서브시스템별 현재 상태

README.md의 STEP 대응표 기준(2026-08-13 시점 갱신):

| 기능 | 상태 |
|---|---|
| SLAM 자율 매핑 | **오늘 실기에서 처음으로 완주 검증됨** (6절 참고). 프론티어 탐색 → 목표 도달 → 자동 저장까지 정상 동작 확인. 장시간/여러 공간 반복 검증은 아직 안 함. |
| Navigation (저장된 지도로 주행) | 코드는 있음(`navigation_runtime_launch.py`), 오늘 세션에서는 안 돌려봄. `mapping-runtime`과 같은 클래스의 버그(아래 6절 TF/cmd_vel_relay 이슈)에 노출돼 있었을 가능성이 있으니, 돌리기 전에 같은 종류의 회귀가 없는지 먼저 점검할 것. |
| YOLO 사람 인식 + Follow Me | 완료된 기능, 이번 세션에서 안 건드림. `전체_실행_순서.txt`가 다루는 게 이 기능. |
| 카메라 텍스처 매핑 (`orchard_mapper`, `map_texture_*`) | 폐기됨. 코드는 남아있지만 활성 파이프라인에서 분리돼 있어야 함. |

---

## 6. 2026-08-13 세션에서 실기로 잡은 버그들 (전부 실기에서만 드러났고, 유닛테스트로는 안 잡히던 것들)

이 세션 전까지 매핑 관련 코드는 **로봇에 한 번도 배포된 적이 없었다** — Mac 저장소에서 커밋/미커밋 상태로만 존재했다. 오늘 처음으로 실제 로봇에 배포 → 빌드 → 실행까지 갔고, 그 과정에서 아래 문제들을 순서대로 만나 고쳤다. 다음에 비슷한 "실기에서 안 됨" 상황을 만나면 이 목록부터 확인할 것.

1. **`Dockerfile`이 `mapping_core.py` 대신 폐기된 텍스처 파일들을 복사하고 있었음.**
   `autonomous_mapping.py`는 `from mapping_core import (...)`를 하는데, `robot_docker/Dockerfile`의 COPY 목록엔 `map_texture_core.py map_texture_recorder.py calibrate_map_texture.py`가 대신 들어가 있었음 → 컨테이너 기동 시 `ModuleNotFoundError`. `mapping_core.py`로 원복. (`ubuntu_v2/tests/test_navigation_recovery.py`, `test_server_image.py`가 이 회귀를 잡아준다 — 반드시 테스트 통과 후 배포할 것.)

2. **`fastdds_profile.xml` 파일 자체가 저장소에 없었음.**
   Dockerfile은 `COPY fastdds_profile.xml ...`을 하는데 파일이 없어서 빌드 자체가 실패. 알고 보니 **로봇 쪽 배포 디렉터리(`/home/pi/robot-control-deploy/`)에는 이미 존재**했음 — 과거 세션에서 로봇에 직접 만들어두고 저장소에 커밋을 안 한 것. 로봇에 있는 실제 버전을 기준으로 저장소에 반영함. **교훈: 로봇 배포 디렉터리와 Mac 저장소가 파일 단위로 따로 놀 수 있다. 배포 전엔 항상 로봇 쪽 실제 파일과 diff 떠볼 것.**

3. **IMU 콜드부트 문제 (2절 참고).**
   증상: `ekf_filter_node`가 계속 `Critical Error, NaNs were detected in the output state` / `TF_NAN_INPUT`을 뱉고, `/imu` 토픽의 orientation/covariance가 전부 0 또는 NaN. `docker restart robot-base-node`로는 절대 안 고쳐짐 (ESP32까지 안 닿음). **로봇 전원을 물리적으로 껐다 켜야만 고쳐짐** — 평평한 바닥에 두고 최소 5초 이상 가만히 두는 절차가 `NAVIGATION_RECOVERY.md`에 이미 문서화돼 있었음. 전원 재인가 후 `/imu`의 `linear_acceleration.z`가 ~9.8 근처로 나오면 정상.

4. **`/tf` vs `/tf_nav` 불일치 — Dockerfile에 남아있던 죽은 `sed` 패치.**
   증상: IMU를 고친 뒤에도 `controller_server`가 계속 `"odom" 프레임이 존재하지 않음`을 반복. 원인: `Dockerfile`에 `RUN sed -i "s|'/tf', 'tf'|'/tf', '/tf_nav'|g" ...`가 남아 있어서 Nav2 내부(`navigation_launch.py` 등)를 `/tf_nav`로 강제 리맵하고 있었는데, `mapping_runtime_launch.py`/`navigation_runtime_launch.py`는 이미 이번 세션 이전 커밋에서 `/tf_nav` 방식을 버리고 그냥 `/tf`를 쓰도록 바뀌어 있었음 (그 이유는 launch 파일 주석에 상세히 설명돼 있음 — FastDDS 디스커버리 레이스 회피). 즉 **launch 파일 레벨 수정은 됐는데 Dockerfile의 sed 패치를 지우는 걸 깜빡한 미완성 리팩터**였음. sed 블록 삭제로 해결.
   **교훈: `/tf` 관련 리맵을 바꿀 땐 launch 파일뿐 아니라 Dockerfile의 `sed`/`RUN` 패치까지 같이 확인할 것 — grep `tf_nav` 전체 저장소.**

5. **`cmd_vel_relay.py`가 Dockerfile COPY 목록에서 빠져 있었음.**
   증상: TF도 정상, SLAM도 지도를 만드는데, `/autonomous_mapping/start`를 호출해도 **로봇이 물리적으로 움직이지 않음**. 원인: `cmd_vel_relay.py`는 파일로는 로봇에 복사됐지만 `Dockerfile`의 COPY 목록에 안 들어가 있어서 컨테이너 안에는 파일이 없었고, `python3 /opt/robot-control/navigation/cmd_vel_relay.py`가 `FileNotFoundError`로 즉사. 이 릴레이가 없으면 Nav2의 `/cmd_vel`이 `/cmd_vel_server`(로봇의 유일한 모터 입력 토픽)로 절대 안 넘어감 — 로그상으론 "탐색 중"이라 정상처럼 보이는 게 함정. Dockerfile에 추가. **회귀 방지 테스트를 `test_navigation_recovery.py`에 추가해둠** (launch 파일 언급뿐 아니라 Dockerfile COPY 목록까지 확인하도록).

**패턴 요약:** 위 5개 중 3개(1, 4, 5)가 전부 "launch 파일/코드는 고쳤는데 Dockerfile을 안 고친" 유형이다. 이 프로젝트에서 뭔가를 리팩터할 때는 **Dockerfile의 COPY 목록과 RUN sed 패치를 항상 같이 grep해서 확인**하는 습관이 필요하다.

---

## 7. 로봇 실기 배포 절차 (모드 A 기준)

```bash
# 1. 로봇 접속 (SSH 키 인증 이미 돼 있음, 비밀번호 불필요)
ssh pi@172.30.1.10   # 또는 raspberrypi.local

# 2. 배포 디렉터리는 git 저장소가 아님 — 파일 단위 rsync/scp로 동기화
#    (Mac 저장소 robot_docker/* 와 로봇의 /home/pi/robot-control-deploy/* 를
#     항상 diff 떠보고 다를 때만 필요한 파일만 scp할 것 — 배포 디렉터리에는
#     .env, backups/, compose.yaml.pre-*.yaml 같은 로봇 전용 파일도 섞여있어서
#     통째로 덮어쓰면 안 됨)

# 3. 이미지 리빌드 (mapping-runtime 예시, base/bridge 등도 같은 이미지 태그 공유)
cd /home/pi/robot-control-deploy
docker compose --profile mapping build mapping-runtime

# 4. 컨테이너 기동 (평소엔 안 뜨는 profile-gated 서비스)
docker compose --profile mapping up -d mapping-runtime

# 5. 로그로 정상 기동 확인 — 아래가 하나도 없어야 정상
docker logs robot-mapping-runtime 2>&1 | grep -i 'nan\|frame does not exist\|died\|No such file'

# 6. 실제 탐색 시작 (⚠️ 로봇이 물리적으로 움직인다 — 안전한 열린 공간, 사람 있는 데서만)
docker exec robot-mapping-runtime bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 service call /autonomous_mapping/start std_srvs/srv/Trigger {}'

# 7. 끝나면 (자동 종료 시 state:"completed"로 바뀜)
docker compose stop mapping-runtime
```

**주의:** `docker compose --profile mapping up -d --force-recreate mapping-runtime`처럼 `--force-recreate`를 쓰면 같은 이미지 태그를 쓰는 `robot-base-node`(IMU/EKF 담당)까지 같이 재생성된다 — IMU 자체는 ESP32 쪽 상태라 안전하지만, 재생성 직후엔 `/imu` 값을 한 번 더 확인해서 정상인지 체크할 것.

---

## 8. 테스트

```bash
cd /Users/yorong/OrX/robot_project
pip3 install --break-system-packages pytest PySide6   # 최초 1회, ROS 의존성 없음
python3 -m pytest ubuntu_v2/tests orchard_mapper/test -q
```

125개 테스트, 전부 통과해야 정상. `rclpy` import가 전혀 없어서 ROS/도커 없이도 Mac에서 그냥 돈다 — **로봇에 배포하기 전에 항상 먼저 로컬에서 이 테스트부터 돌릴 것.** 특히 `test_navigation_recovery.py`, `test_server_image.py`는 Dockerfile/launch 파일의 텍스트 내용을 직접 assert하는 회귀 테스트라 6절 같은 버그를 미리 잡아준다.

---

## 9. 지금 커밋 안 된 변경사항 (2026-08-13 세션 종료 시점)

```
 M robot_docker/Dockerfile                     # mapping_core.py 복구 + tf_nav sed 삭제 + cmd_vel_relay.py 추가
 M robot_docker/compose.yaml
 M robot_docker/mapping_runtime_launch.py
 M robot_docker/navigation_runtime_launch.py
 M robot_docker/robot_cmd_bridge.py
 M ubuntu_v2/desktop_gui/main.py
 M ubuntu_v2/docker/Dockerfile.compute
 M ubuntu_v2/robot_app/robot_gateway.py
 M ubuntu_v2/tests/test_navigation_recovery.py     # 회귀 테스트 추가/갱신
 M ubuntu_v2/tests/test_robot_cmd_bridge.py         # 픽스처 보강
?? robot_docker/cmd_vel_relay.py                    # 신규 파일
?? robot_docker/fastdds_profile.xml                 # 신규 파일 (로봇 실물 버전과 동기화됨)
```

이 상태 그대로 로봇에 배포해서 **자율 매핑 완주 + 지도 저장까지 실기 검증 완료**. 아직 `git commit` 안 했음 — 다음 세션에서 커밋할 때는 위 6절의 버그 5개를 커밋 메시지에 요약하는 걸 권장.

---

## 10. 다음에 할 일 (우선순위 순)

1. **git commit** — 위 9절 변경사항. 아직 아무도 커밋 안 함.
2. **Navigation runtime 재검증** — 오늘은 mapping만 돌려봤다. `navigation_runtime_launch.py`도 같은 종류의 TF/cmd_vel_relay 회귀가 있었는지 실기로 확인 필요 (Dockerfile은 이미 고쳐졌으니 이번엔 될 가능성 높음, 그래도 검증은 필요).
3. **로봇 배포 디렉터리 ↔ Mac 저장소 완전 동기화 점검** — 6절 2번 사례처럼 로봇에만 있고 저장소엔 없는 파일이 더 있을 수 있다. `robot_docker/*`와 로봇의 `/home/pi/robot-control-deploy/*`를 파일 단위로 전수 diff 떠서 확인 권장.
4. **장시간/여러 공간 매핑 반복 검증** — README STEP 2에 적힌 "장시간 현장검증 필요"가 아직 유효. 오늘은 좁은 공간에서 1회 완주만 확인.
5. **컴퓨트 서버 모드(모드 B) 실기 검증** — 코드/Zenoh 설정은 있지만 실제로 별도 서버를 띄워서 끝까지 테스트해본 적은 이번 세션에서도 없음.
6. **`전체_실행_순서.txt` 갱신 또는 폐기 표시** — 지금 로봇 단독 모드(4절) 기준으로 다시 쓰거나, 파일 상단에 "구버전, 참고만" 경고를 명시적으로 추가.
7. **폐기된 텍스처 매핑 파일 정리 여부 논의** — `map_texture_*.py`, `calibrate_map_texture.py`, `obstacle_texture_fusion.py`, `orchard_mapper/`를 완전히 삭제할지, 별도 실험 트랙으로 유지할지 사용자와 확인.

---

## 11. 참고 문서 (이 문서와 같이 읽을 것)

- `README.md` — 프로젝트 전체 개요, STEP 대응표, 지도 생성 원칙.
- `NAVIGATION_RECOVERY.md` — Navigation 단독 완주 검증용 복구 절차. **IMU 콜드부트 절차(6절 3번)의 원출처**이자, 백업 우선 원칙("재생성보다 백업이 먼저") 등 실기 작업 시 안전수칙이 잘 정리돼 있음.
- `전체_실행_순서.txt` — ⚠️ 구버전, 4절 설명 참고하고 그대로 따르지 말 것.
