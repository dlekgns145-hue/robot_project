#!/bin/bash
# smart_agent.sh
# ------------------------------------------------------------
# start_agent_rpi5.sh를 매번 실행하면 micro-ros-agent 컨테이너가 중복 생성됩니다.
# 이 스크립트는 이미 떠있는 게 있으면 그냥 그걸 쓰고, 없을 때만 새로 시작합니다.
#
# 사용법 (기존 'sh ~/start_agent_rpi5.sh' 대신 이걸 쓰세요):
#   sh ~/smart_agent.sh
# ------------------------------------------------------------

IMAGE="microros/micro-ros-agent:humble"

EXISTING=$(docker ps -qf ancestor=$IMAGE | head -1)

if [ -n "$EXISTING" ]; then
    echo "[smart_agent] micro-ROS agent 이미 실행 중입니다 (컨테이너: $EXISTING)"
    echo "[smart_agent] 새로 안 켭니다. 로그 보려면: docker logs -f $EXISTING"
else
    echo "[smart_agent] agent가 없어서 새로 시작합니다..."
    sh ~/start_agent_rpi5.sh
fi
