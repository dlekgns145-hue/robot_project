"""
robot_sender.py - 통신 테스트용 (노트북에서 실행)
-------------------------------------------------
robot_cmd_bridge.py가 로봇 Docker 컨테이너에서 켜져 있는 상태에서,
간단히 전진 명령 하나를 보내서 통신이 되는지만 확인하는 스크립트.

실행 전 확인:
  - Wi-Fi가 Micro_ros에 연결되어 있어야 함
  - robot_cmd_bridge.py가 로봇 쪽에서 "cmd_bridge ready. listening on 9999" 상태여야 함
  - ⚠️ 로봇을 받침대에 올려 바퀴가 공중에 뜨게 한 상태에서 테스트할 것

실행:
    python robot_sender.py
"""

import socket
import json
import time

ROBOT_IP = "172.30.1.76"
ROBOT_PORT = 9999


def send_cmd(sock, linear, angular):
    msg = json.dumps({"linear": linear, "angular": angular}) + '\n'
    sock.sendall(msg.encode('utf-8'))


def main():
    print(f'{ROBOT_IP}:{ROBOT_PORT} 연결 시도...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((ROBOT_IP, ROBOT_PORT))
    print('연결 성공! 2초간 전진 명령 전송...')

    send_cmd(sock, 0.2, 0.0)
    time.sleep(2)
    send_cmd(sock, 0.0, 0.0)

    print('정지 명령 전송 완료. 바퀴가 2초 돌다 멈췄으면 통신 성공!')
    sock.close()


if __name__ == '__main__':
    main()
