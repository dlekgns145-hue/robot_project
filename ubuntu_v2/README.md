# Robot Control v2

데스크톱 GUI, Ubuntu VM gateway, Raspberry Pi 자동시작을 묶은
현재 운영 구조입니다.

GUI는 두 종류로 분리되어 있습니다.

- `desktop_gui`: 설정, 비전 튜닝, Navigation을 포함한 관리자용 GUI
- `user_gui`: 카메라, Follow Me, 간편 운전 중심의 일반 사용자용 GUI

사용자용 GUI의 설치와 실행은 `user_gui/사용자_GUI_실행법.txt`를 확인하세요.

## 최종 아키텍처

```text
Windows/macOS GUI
  - 로봇 MJPEG 카메라(기본) 또는 로컬 카메라
  - YOLO
  - 수동/Follow 명령
        │ TCP 9999 (고정 대상: Ubuntu VM)
        ▼
Ubuntu VM Docker: gateway
  - GUI 토큰 검증
  - 로봇 Wi-Fi MAC → 현재 DHCP IP 탐색
  - UTM NAT에서는 GUI가 해석한 raspberrypi.local IP를 보조 전달
  - IP 변경/연결 끊김 시 자동 재탐색
        │ TCP 9999 (현재 로봇 IP로 자동 연결)
        ▼
실물 로봇 Raspberry Pi
  - 기존 robot_cmd_bridge.py
  - 기존 micro-ROS Agent/base_node_X3
  - /cmd_vel, LiDAR, 서보 및 모터
```

GUI는 항상 Ubuntu VM에만 연결합니다. GUI 설정에는 로봇 IP를 입력하지
않습니다. 로봇 IP가 DHCP로 변경되어도 고정된 Wi-Fi MAC 주소로 다시 찾습니다.

MAC 주소는 TCP 접속 주소가 아닙니다. 게이트웨이가 같은 LAN에서 ARP를 이용해
`고정 MAC → 현재 IP`를 찾을 때 사용하는 로봇 식별자입니다.

GUI도 `raspberrypi.local`을 5초마다 다시 해석해 인증된 연결로 VM에 현재 IP를
알려줍니다. 따라서 UTM의 Shared/NAT 환경처럼 VM에서 직접 MAC 탐색이 불가능해도
VM에서 로봇 IP로 나가는 TCP 통신만 가능하면 재연결할 수 있습니다.

## 1. 네트워크 전제조건

Ubuntu VM에서 로봇까지 먼저 통신되어야 합니다.

```bash
ping -c 3 raspberrypi.local
nc -vz raspberrypi.local 9999
```

VM에서 로봇 IP가 `FAILED`로 보이면 VM 내부의 MAC 직접 탐색은 사용할 수 없습니다.
MAC/ARP는 같은 Layer-2 네트워크에서만 동작합니다. UTM의 macOS Wi-Fi 브리지가
가상 MAC을 차단하는 환경에서는 다음 중 하나가 필요합니다.

- UTM을 Shared Network로 되돌리고 VM에서 로봇 현재 IP의 TCP 9999 접근 확인;
- USB Wi-Fi 어댑터를 Ubuntu VM에 직접 연결;
- Mac을 유선 Ethernet에 연결하고 해당 인터페이스로 VM을 브리지;
- 회사 공유기에서 로봇/VM MAC에 DHCP 예약과 단말 간 통신 허용.

공유기 DHCP 예약이 가능하면 로봇과 VM 모두 예약하는 것이 가장 안정적입니다.
게이트웨이의 MAC 탐색은 IP가 예기치 않게 바뀌는 경우를 위한 복구 장치입니다.

## 2. 로봇 Wi-Fi MAC 확인

현재 확인된 이 로봇의 네트워크 값은 다음과 같습니다.

```text
wlan0  회사 Wi-Fi  MAC 2c:cf:67:7b:48:d7  (DHCP 172.30.1.x)
wlan1  Micro_ros   MAC 20:e1:5d:f7:a9:af  (고정 10.42.0.1)
```

기본 게이트웨이 설정에는 회사 Wi-Fi용 `wlan0` MAC을 사용합니다. 이 값은
`.env.robot-ready`에 이미 반영되어 있습니다.

Mac에서 `raspberrypi.local`이 확인되므로 로봇을 분해할 필요 없이 SSH로 확인할
수 있습니다.

```bash
ssh <로봇계정>@raspberrypi.local
ip -br link
```

실제 Wi-Fi 인터페이스가 `wlan0`이면:

```bash
cat /sys/class/net/wlan0/address
```

외장 USB Wi-Fi는 `wlx...` 이름일 수 있습니다. 이때는 출력에 표시된 실제 이름을
사용합니다.

```bash
cat /sys/class/net/wlx실제이름/address
```

Ethernet MAC이 아니라 현재 회사 Wi-Fi에 연결된 어댑터의 MAC을 사용해야 합니다.

Mac의 ARP 표에서도 확인할 수 있습니다.

```bash
ping -c 1 raspberrypi.local
arp -a | grep raspberrypi
```

## 3. 로봇 기존 서비스 시작

로봇의 기존 Yahboom ROS 환경은 그대로 사용합니다. 로봇 터미널에서:

```bash
sh ~/smart_agent.sh
```

다른 로봇 터미널에서 ROS 컨테이너와 base node를 확인합니다.

```bash
sh ~/smart_ros2.sh
ros2 node list
```

`/YB_Car_Node`가 이미 보이면 `base_node_X3`를 중복 실행하지 않습니다. 보이지
않을 때만 실행합니다.

```bash
ros2 run yahboomcar_base_node base_node_X3
```

다른 로봇 터미널에서 기존 TCP 브리지를 실행합니다.

```bash
sh ~/start_bridge.sh
```

Mac에서 확인합니다.

```bash
nc -vz raspberrypi.local 9999
```

기존 로봇 브리지는 명령만 받고 상태 응답을 보내지 않습니다. Ubuntu VM의 새
gateway가 기존 형식으로 명령을 변환하고 GUI용 연결 상태를 생성합니다.

## 4. Ubuntu VM 설정

이 로봇용 준비 파일을 사용하면 MAC을 다시 입력할 필요가 없습니다.

```bash
cd ~/ubuntu_v2
cp .env.robot-ready .env
```

또는 첫 실행 스크립트가 `.env` 생성부터 Docker 실행까지 처리합니다.

```bash
chmod +x scripts/first_run_vm.sh
./scripts/first_run_vm.sh
```

수동으로 설정하려면:

```bash
cd ~/ubuntu_v2
cp .env.example .env
nano .env
```

필수 설정:

```dotenv
ROBOT_MAC=dc:a6:32:12:34:56
ROBOT_HOST=raspberrypi.local
ROBOT_IP=
ROBOT_PORT=9999

COMMAND_PORT=9999
COMMAND_TOKEN=긴-임의의-비밀값
```

`ROBOT_IP`는 비워둡니다. 값을 넣으면 MAC 탐색보다 고정 IP가 우선되므로 DHCP
변경을 자동으로 따라가지 않습니다. `ROBOT_HOST`는 MAC 탐색 실패 시 사용하는
mDNS 보조 수단입니다.

인터페이스 자동 탐색이 실패할 때만 다음처럼 지정합니다.

```dotenv
ROBOT_INTERFACE=enp0s1
ROBOT_DISCOVERY_CIDR=172.30.1.0/24
```

방화벽에서 GUI가 접속할 TCP 포트를 허용합니다.

```bash
sudo ufw allow 9999/tcp
```

### 같은 Wi-Fi의 다른 PC에서 UTM Ubuntu에 접속

UTM의 Shared Network를 사용하면 `192.168.64.x` VM 주소는 Mac에서만 접근할 수
있습니다. VM 네트워크 모드를 바꾸는 대신 Mac에 LAN 프록시를 설치하면 로봇 탐색
구성은 그대로 두고 다른 PC에서도 접속할 수 있습니다. Ubuntu gateway가 실행된
상태에서 **Mac 터미널**에 입력합니다.

```bash
cd ~/robot_project
bash ubuntu_v2/scripts/install_macos_lan_proxy.sh 192.168.64.15 <다른-PC-IP>
```

스크립트가 출력한 Mac LAN IP와 `9999`를 다른 PC의 GUI에 입력하고, 기존과 같은
제어 토큰을 사용합니다. 프록시는 Mac의 현재 Wi-Fi 주소에서만 수신하고 명령에 넣은
다른 PC의 IP 한 대만 허용하며, 로그인 및 재부팅 후 자동으로 시작됩니다. macOS
방화벽 확인 창이 나타나면 Python의 수신 연결을 허용해야 합니다.

현재 설정과 로그 확인:

```bash
launchctl print gui/$(id -u)/com.robot-project.ubuntu-lan-proxy
tail -f ~/Library/Logs/robot-control-lan-proxy.log
```

제거:

```bash
bash ubuntu_v2/scripts/uninstall_macos_lan_proxy.sh
```

안전을 위해 제어 gateway는 동시에 여러 운전자를 허용하지 않습니다. 다른 PC로
제어권을 옮길 때는 기존 GUI의 연결을 먼저 해제합니다.

## 5. Ubuntu Docker 실행

기본 실행에는 gateway 하나만 올라옵니다.

```bash
cd ~/ubuntu_v2
docker compose config
docker compose build gateway
docker compose up -d gateway
docker compose ps
docker compose logs -f gateway
```

정상 로그:

```text
robot gateway listening through TCP :9999
robot bridge connected: 172.30.1.x:9999 via mac-arp-scan
```

로봇 IP가 바뀌거나 전원이 늦게 켜지면 gateway는 자동으로 다시 탐색합니다.

진단:

```bash
docker compose exec gateway ip -4 route
docker compose exec gateway ip -4 neigh
docker compose exec gateway arp-scan --interface=enp0s1 --localnet
```

마지막 명령 결과에 `.env`의 `ROBOT_MAC`이 보여야 MAC 자동 탐색이 가능합니다.

기존 `micro-ros-agent`, `base-node`, ROS 직접 브리지는 기본 실행에서 제외했습니다.
ESP32가 VM Agent에 직접 접속하는 별도 로봇에서만 다음 프로필을 사용합니다.

```bash
docker compose --profile direct-hardware up -d
```

현재 Raspberry Pi 탑재 로봇에서는 이 프로필을 사용하지 않습니다.

## 6. VM 부팅 시 자동 시작

직접 실행이 성공한 다음:

```bash
cd ~/ubuntu_v2
sudo ./scripts/install_vm_ubuntu.sh
sudo systemctl status robot-control-v2
```

`install_vm_ubuntu.sh`는 gateway 이미지를 빌드하고 서비스를 즉시
시작합니다. 이후 Ubuntu가 부팅되면 gateway도 자동으로 실행됩니다.

## 6-1. 로봇 Raspberry Pi 부팅 시 자동 시작

로봇 런타임은 systemd 스크립과 `--rm` 컨테이너를 사용하지 않습니다.
Yahboom ROS 이미지에 브리지와 카메라 코드를 구운 사용자 이미지를
만들고 Docker Compose만 전체 생명주기를 관리합니다.

Mac에서 한 번만 배포합니다.

```bash
ssh pi@172.30.1.18 'mkdir -p ~/robot-control-deploy'
scp robot_docker/* pi@172.30.1.18:~/robot-control-deploy/
ssh pi@172.30.1.18
sudo bash ~/robot-control-deploy/install.sh
```

설치 스크립트는 기존 중복 컨테이너와 `robot-control-*` systemd
서비스를 제거하고 다음 고정 컨테이너 이름을 사용합니다.

- `robot-microros-agent`
- `robot-base-node`
- `robot-command-bridge`
- `robot-camera-stream`

확인:

```bash
cd ~/robot-control-deploy
docker compose ps
docker compose logs --tail=100
```

로봇 MCU 시리얼은 `robot-microros-agent`만 소유합니다. `robot-base-node`는
`/odom_raw` 같은 ROS 토픽을 소비하므로 같은 시리얼 장치를 base 컨테이너에도
마운트하면 안 됩니다. 설치 스크립트는 CP210x의 `/dev/serial/by-id` 경로를
자동으로 찾아 `robot-control-deploy/.env`에 저장하고 컨테이너 안의
`/dev/robot-controller`로 명시적으로 전달합니다.

수동 실행할 때는 `.env.example`을 참고하여 실제 장치 경로를 지정할 수 있습니다.

```dotenv
ROBOT_SERIAL_DEVICE=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
ROBOT_SERIAL_BAUD=921600
```

Agent 재생성 후 로그가 `running... fd: 3`에서 멈추면 MCU의 RESET 버튼을 한 번
눌러야 합니다. 정상일 때는 `create_client`, `session established`,
`create_subscriber`가 이어서 출력됩니다.

```bash
docker logs -f robot-microros-agent
docker logs -f robot-command-bridge
```

`robot-command-bridge`는 `/cmd_vel` subscriber가 없거나 `/scan` publisher가
없으면 상태 변화 시 오류를 남깁니다. 기존 `yahboom_ros_main` 및
`smart_ros2.sh`를 새 Compose 구성과 동시에 실행하지 않습니다.

### Yahboom MS200 LiDAR 실행 구조

Yahboom 공식 MicroROS-Car-Pi5 구성에는 별도의 USB LiDAR 드라이버 실행 명령이
없습니다. 제어보드가 micro-ROS Agent에 연결된 뒤 radar/IMU 원시 데이터를
발행하고, 공식 실행 명령인 아래 bringup이 base 처리와 scan/odom 후처리 노드를
시작합니다.

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
```

새 `robot-base-node`는 단독 `base_node_X3` 대신 위 bringup을 실행합니다. 설치된
Yahboom 이미지에 bringup 패키지가 없을 때만 `base_node_X3`로 자동 대체합니다.
따라서 bringup 실행 후에도 `/scan` 메시지가 오지 않으면 별도 Docker 장치 마운트
문제가 아니라 ESP32의 radar publish 상태를 확인해야 합니다.

```bash
docker logs --tail=100 robot-base-node
docker exec robot-base-node bash -lc \
  'source /opt/ros/humble/setup.bash; ros2 topic hz /scan'
```

브리지는 이제 `/scan` publisher 존재 여부만 보지 않고 최근 2초 내 실제
`LaserScan` 메시지가 수신됐는지도 확인합니다.

## 7. GUI 실행

Windows 설치:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup_gui_windows.ps1
```

그다음 `run_gui_windows.vbs`를 실행합니다.

macOS 설치:

```bash
cd ubuntu_v2/desktop_gui
./setup_gui_macos.sh
```

그다음 `Robot Control v2.app` 또는 `run_gui_macos.command`를 실행합니다.

GUI는 저장된 Ubuntu VM IP가 있으면 시작 직후 자동으로 연결하고,
연결이 끊기면 백그라운드에서 계속 재연결합니다.

### 관리자 운영 현황과 작업 캘린더

관리자용 `desktop_gui`에는 다음 세 페이지가 있습니다.

- `실시간 제어`: 기존 카메라, Follow Me, Navigation, 수동 주행;
- `운영 현황`: 로봇별 온라인 상태, 최근 작업 세션, 켜짐/꺼짐 감지 로그;
- `작업 캘린더`: 날짜별 작업 여부와 누적 작업시간.

운영 기록은 GUI 컴퓨터가 아니라 항상 실행되는 Ubuntu gateway의 SQLite에
저장됩니다. 따라서 관리자 GUI가 꺼져 있어도 gateway가 로봇 연결 변화를
감지한 시간은 계속 기록됩니다.

```text
ubuntu_v2/data/operations.sqlite3
```

Docker 안에서는 `/var/lib/robot-control-v2/operations.sqlite3`로 마운트됩니다.
gateway 컨테이너를 재생성해도 호스트의 `data` 디렉터리는 보존됩니다. 이 기록의
켜짐/꺼짐은 물리 전원 센서값이 아니라 Raspberry Pi command bridge의
온라인/오프라인 감지 시각입니다.

로봇 표시 이름과 영구 식별자는 `.env`에서 설정합니다.

```dotenv
ROBOT_ID=yahboom-pi5-01
ROBOT_NAME=Yahboom Pi5 Robot 1
```

현재 gateway는 한 로봇을 관리하지만 DB와 관리자 표는 여러 `ROBOT_ID`를
수용할 수 있는 형태입니다. 여러 로봇 동시 관리는 gateway의 relay 인스턴스를
로봇별로 확장하는 후속 작업이 필요합니다.

실물 로봇 없이 화면을 확인하려면 `운영 현황` 탭의
`샘플 데이터 미리보기`를 누릅니다. 샘플은 DB에 저장되지 않습니다.

GUI 연결값:

```text
Ubuntu VM IP: VM의 현재 주소
명령 포트: 9999
제어 토큰: VM .env의 COMMAND_TOKEN
로봇 이름(보조 탐색): raspberrypi.local
```

위 Raspberry Pi Docker 런타임을 설치하면 MJPEG 카메라 서버도
부팅 시 자동으로 실행됩니다. 수동 진단:

```bash
docker logs --tail=100 robot-camera-stream
```

`camera stream ready on 0.0.0.0:8080`이 표시되면 GUI의 카메라
입력에서 `로봇 카메라 (자동)`을 선택합니다. GUI는 gateway가 알려준
현재 로봇 IP로 `http://<로봇 IP>:8080/stream.mjpg`를 자동 구성합니다.

GUI 상태는 두 단계를 구분합니다.

- `Ubuntu VM 게이트웨이 연결됨`: GUI와 VM 연결 성공;
- `VM · 로봇 172.30.1.x 연결됨`: VM과 실물 로봇까지 연결 성공.

로봇 연결이 완료된 다음에만 Follow와 수동 주행을 시작합니다. 최초 테스트는
바퀴를 공중에 띄운 상태에서 낮은 속도로 진행합니다.

GUI 기능 버튼:

- `Perception`: 선택한 카메라와 YOLO로 영상/사람 인식만 실행하고 주행 명령은 보내지 않습니다.
- `Follow Me`: Perception을 함께 시작하고 VM gateway로 추적 주행 명령을 보냅니다.
- `Navigation`: ROS2/Nav2가 설치된 환경에서 통합 `integrated_main` 프로세스를 시작합니다.
- `전체 중지`: 영상 추론과 Navigation 목표를 중지하고 정지 명령을 보냅니다.

Follow Me와 Navigation은 동시에 로봇을 제어하지 않도록 서로 전환됩니다.
맥에는 기본적으로 ROS2/Nav2가 없으므로 Navigation 버튼은 ROS 워크스테이션에서
GUI를 실행했을 때 활성화됩니다.

## 8. 테스트

ROS, Docker 또는 카메라가 필요 없는 테스트:

```bash
python3 -m unittest discover -s ubuntu_v2/tests -v
```
