#!/bin/bash
# cleanup_containers.sh
# ------------------------------------------------------------
# ros-humble, micro-ros-agent 각각 여러 개 떠있을 때,
# "가장 최근에 만든 것 1개"만 남기고 나머지는 정지시킵니다.
#
# 사용법:
#   sh ~/cleanup_containers.sh
# ------------------------------------------------------------

cleanup_image() {
    IMAGE=$1
    # CreatedAt 기준 최신순 정렬, 1번째(최신)는 남기고 나머지 정지
    IDS=$(docker ps -qf ancestor=$IMAGE)
    COUNT=$(echo "$IDS" | grep -c .)

    if [ "$COUNT" -le 1 ]; then
        echo "[cleanup] $IMAGE : 정리할 것 없음 (현재 $COUNT개)"
        return
    fi

    KEEP=$(docker ps -qf ancestor=$IMAGE --format '{{.ID}} {{.CreatedAt}}' | sort -k2 -r | head -1 | awk '{print $1}')
    echo "[cleanup] $IMAGE : $COUNT개 발견, 최신 컨테이너($KEEP)만 남기고 나머지 정지"

    for id in $IDS; do
        if [ "$id" != "$KEEP" ]; then
            docker stop "$id"
            echo "  - 정지됨: $id"
        fi
    done
}

cleanup_image "yahboomtechnology/ros-humble:4.1.2"
cleanup_image "microros/micro-ros-agent:humble"

echo ""
echo "[cleanup] 정리 후 현재 상태:"
docker ps
