# Robot Control v2

기존 `local_test/`와 `robot_project/`를 건드리지 않고 새로 만든 실행 구조입니다.

## 최종 아키텍처

```text
Windows/macOS GUI
  - 로컬 카메라
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
sudo nano /opt/robot-control-v2/.env
cd /opt/robot-control-v2
sudo docker compose build gateway
sudo systemctl start robot-control-v2
sudo systemctl status robot-control-v2
```

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

GUI 연결값:

```text
Ubuntu VM IP: VM의 현재 주소
명령 포트: 9999
제어 토큰: VM .env의 COMMAND_TOKEN
로봇 이름(보조 탐색): raspberrypi.local
```

GUI 상태는 두 단계를 구분합니다.

- `Ubuntu VM 게이트웨이 연결됨`: GUI와 VM 연결 성공;
- `VM · 로봇 172.30.1.x 연결됨`: VM과 실물 로봇까지 연결 성공.

로봇 연결이 완료된 다음에만 Follow와 수동 주행을 시작합니다. 최초 테스트는
바퀴를 공중에 띄운 상태에서 낮은 속도로 진행합니다.

## 8. 테스트

ROS, Docker 또는 카메라가 필요 없는 테스트:

```bash
python3 -m unittest discover -s ubuntu_v2/tests -v
```
