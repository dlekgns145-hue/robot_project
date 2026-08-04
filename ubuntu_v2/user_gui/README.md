# Robot Companion 사용자용 GUI

관리자용 `../desktop_gui`는 그대로 두고, 일상 사용에 필요한 기능만 제공하는
별도 데스크톱 앱입니다.

## 포함 기능

- 저장된 Ubuntu VM 주소로 자동 연결
- 로봇 연결 상태와 현재 모드 표시
- 로봇 카메라 및 사람 인식 화면
- 고정된 안전 설정의 Follow Me
- 누르는 동안만 동작하는 방향 버튼과 키보드 운전
- 3단계 속도 선택과 긴급 정지

YOLO 경로, 추론 간격, Navigation 좌표, 상세 텔레메트리와 로그는 관리자용
GUI에만 남겨두었습니다.

## 빠른 실행

Windows 첫 실행:

```text
1_INSTALL_AND_RUN_WINDOWS.bat 더블클릭
```

Windows 재실행:

```text
run_user_gui_windows.vbs 더블클릭
```

macOS 첫 실행:

```text
1_INSTALL_AND_RUN_MAC.command 더블클릭
```

macOS 재실행:

```text
run_user_gui_macos.command 더블클릭
```

자세한 내용은 `사용자_GUI_실행법.txt`를 확인하세요.

## 코드 구성

`main.py`와 `theme.py`는 사용자 화면 전용입니다. 실제 로봇 통신,
연결 종료 시 정지 버스트, 사람 추적 로직은 관리자 GUI의 `robot_client.py`,
`control_logic.py`, `vision_worker.py`를 재사용합니다. 따라서 배포할 때는
`user_gui` 폴더만 떼어내지 말고 `ubuntu_v2` 폴더 구조를 유지해야 합니다.
