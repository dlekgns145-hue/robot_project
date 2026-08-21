#!/usr/bin/env python3
"""
무거운 프로파일 컨테이너(mapping-runtime/navigation-runtime) 기동 전용 로컬
런처 (호스트에서 직접 실행, 컨테이너 아님)
------------------------------------------------------------------------------
robot_cmd_bridge.py는 도커 컨테이너 안에서 돌아가기 때문에 호스트의 도커를
직접 조작할 권한이 없다 -- 일부러 없앤 것이다. docker.sock을 컨테이너에
그대로 물리면 그 컨테이너가 호스트 도커 전체(=사실상 호스트 자체)를 조작할
수 있게 되어, 통제 브릿지 하나가 뚫리면 로봇 호스트까지 뚫리는 셈이 된다.

대신 이 스크립트는 systemd로 호스트에서 직접 도는 별도 프로세스로, 정해진
액션 몇 가지(아래 ACTION_COMPOSE_ARGS)만 받는 좁은 유닉스 도메인 소켓을 연다.
브릿지 컨테이너에는 이 소켓 파일 하나만 마운트하고, 여기서 받아들이는
액션도 하드코딩된 것뿐이라 브릿지가 이 소켓을 통해 할 수 있는 일은 정확히
"이 컨테이너들 시작/정지"로만 좁혀진다. mapping-runtime과 navigation-runtime은
둘 다 profiles: [mapping]/[navigation]으로 기본 비활성화돼 있어(아무도 안
시켰는데 로봇이 알아서 움직이면 안 되므로) 실제로 필요할 때만 이 런처를
거쳐 띄운다.
"""

import json
import os
import socket
import subprocess

SOCKET_PATH = os.environ.get(
    "MAPPING_LAUNCHER_SOCKET", "/home/pi/robot-control-deploy/mapping_launcher.sock"
)
DEPLOY_DIR = os.environ.get("ROBOT_DEPLOY_DIR", "/home/pi/robot-control-deploy")
COMPOSE_TIMEOUT_SEC = 60


ACTION_COMPOSE_ARGS = {
    "start_mapping_runtime": ["up", "-d", "mapping-runtime"],
    "stop_mapping_runtime": ["stop", "mapping-runtime"],
    "start_navigation_runtime": ["up", "-d", "navigation-runtime"],
    "stop_navigation_runtime": ["stop", "navigation-runtime"],
}


def handle_request(payload: dict) -> dict:
    compose_args = ACTION_COMPOSE_ARGS.get(payload.get("action"))
    if compose_args is None:
        return {"ok": False, "error": f"unsupported action: {payload.get('action')!r}"}
    try:
        result = subprocess.run(
            ["docker", "compose", *compose_args],
            cwd=DEPLOY_DIR,
            capture_output=True,
            text=True,
            timeout=COMPOSE_TIMEOUT_SEC,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"ok": False, "error": str(error)}
    if result.returncode != 0:
        return {"ok": False, "error": result.stderr.strip() or "docker compose failed"}
    return {"ok": True}


def main():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    # 이 소켓의 진짜 방어선은 "액션 한 종류만 하드코딩"이지, 파일 권한이
    # 아니다 -- 브릿지 컨테이너는 root로 돌 수도 있어서 소유자를 좁혀봤자
    # 의미가 없다.
    os.chmod(SOCKET_PATH, 0o666)
    server.listen(4)
    print(f"runtime_launcher listening on {SOCKET_PATH}", flush=True)
    while True:
        conn, _ = server.accept()
        try:
            data = conn.recv(4096)
            if not data:
                continue
            try:
                payload = json.loads(data.decode("utf-8"))
            except json.JSONDecodeError:
                response = {"ok": False, "error": "invalid json"}
            else:
                response = handle_request(payload)
            conn.sendall(json.dumps(response).encode("utf-8"))
        except Exception as error:  # noqa: BLE001 -- 리스너 자체는 절대 죽으면 안 됨
            print(f"request handling error: {error}", flush=True)
        finally:
            conn.close()


if __name__ == "__main__":
    main()
