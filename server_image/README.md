# Robot compute server image

이 폴더를 포함한 배포 압축 파일을 Ubuntu 22.04, 24.04 또는 26.04 PC에 복사한 뒤
다음 명령 하나로 설치합니다. `amd64`와 `arm64`를 지원합니다.

```bash
tar -xzf robot-control-server-*.tar.gz
cd robot-control-server-*
sudo ./INSTALL_SERVER.sh
```

설치 프로그램은 Docker Engine과 Compose plugin이 없으면 Docker 공식 저장소에서
설치하고, gateway와 지도 후처리 이미지를 빌드한 다음 systemd에
등록합니다. 이후 PC를 재부팅해도 서버가 자동으로 시작됩니다.

인터넷이 없는 Ubuntu에 옮기려면 먼저 Docker가 실행되는 같은 CPU 아키텍처의
컴퓨터에서 `server_image/export_docker_images.sh`를 실행한 뒤
`server_image/build_server_image.sh`를 다시 실행합니다. 생성되는 큰 배포 파일에는
Docker 이미지 레이어까지 포함되어 대상 PC에서 다시 다운로드하거나 빌드하지
않습니다. Docker Engine 자체 패키지는 대상 PC에 미리 설치되어 있어야 합니다.

기본값은 현재 로봇에 맞춰져 있습니다.

```text
로봇 IP  172.30.1.10
로봇 MAC 2c:cf:67:7b:48:d7
GUI 포트 9999
```

다른 값으로 설치하려면 다음처럼 지정합니다.

```bash
sudo ./INSTALL_SERVER.sh \
  --robot-ip 172.30.1.10 \
  --robot-mac 2c:cf:67:7b:48:d7 \
  --token 'change-this-to-a-long-private-token'
```

GUI가 서버와 같은 PC에 있으면 `127.0.0.1:9999`, 다른 PC에 있으면 Ubuntu
서버의 LAN IP와 포트 `9999`를 사용합니다. 토큰 확인:

```bash
sudo sed -n 's/^COMMAND_TOKEN=//p' \
  /opt/robot-control-server/ubuntu_v2/.env
```

상태와 로그:

```bash
sudo systemctl status robot-control-server
cd /opt/robot-control-server/ubuntu_v2
sudo docker compose --profile compute ps
sudo docker compose --profile compute logs -f gateway map-postprocessor
```

원본 지도, 보정 지도, 처리 보고서와 운영 기록은 재설치해도 보존됩니다.
보정 작업은 지도와 지도 저장 시점의 로봇 자세를 같은 좌표변환으로
처리합니다. 결과 묶음은 로봇에 다시 전송되고, 다음 Navigation 시작 시
보정 PGM/YAML과 보정 AMCL 초기 자세가 동시에 적용됩니다.

```text
/opt/robot-control-server/ubuntu_v2/maps
/opt/robot-control-server/ubuntu_v2/data
```

제거 스크립트는 컨테이너와 자동 시작 서비스만 내리고 위 데이터는 지우지
않습니다.

```bash
sudo ./UNINSTALL_SERVER.sh
```
