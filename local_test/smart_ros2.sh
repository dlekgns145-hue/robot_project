#!/bin/bash
# smart_ros2.sh
# ------------------------------------------------------------
# ./ros2_humble.sh를 무작정 실행하면 매번 새 컨테이너가 생깁니다.
# 이 스크립트는 먼저 "이미 떠 있는 ros-humble 컨테이너가 있는지" 확인해서,
#   - 있으면 -> 그 컨테이너 안으로 그냥 들어감 (docker exec)
#   - 없으면 -> 그때만 ./ros2_humble.sh 로 새로 만듦
#
# 사용법 (기존 './ros2_humble.sh' 대신 이걸 쓰세요):
#   sh ~/smart_ros2.sh
# ------------------------------------------------------------

IMAGE="yahboomtechnology/ros-humble:4.1.2"

EXISTING=$(docker ps -qf ancestor=$IMAGE | head -1)

if [ -n "$EXISTING" ]; then
    echo "[smart_ros2] 이미 떠있는 컨테이너 발견: $EXISTING"
    echo "[smart_ros2] 새로 만들지 않고 그 컨테이너로 들어갑니다."
    docker exec -it "$EXISTING" bash
else
    echo "[smart_ros2] 떠있는 컨테이너가 없습니다. 새로 만듭니다..."
    cd ~ && ./ros2_humble.sh
fi
