#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker Engine가 설치되어 있지 않습니다." >&2
    exit 1
fi
if ! docker compose version >/dev/null 2>&1; then
    echo "Docker Compose plugin이 설치되어 있지 않습니다." >&2
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "현재 사용자가 Docker daemon에 접근할 수 없습니다." >&2
    echo "로그아웃/로그인 후 다시 실행하거나 Docker 권한을 확인하세요." >&2
    exit 1
fi

if [[ ! -f .env ]]; then
    cp .env.robot-ready .env
    echo "로봇 MAC이 적용된 .env를 생성했습니다."
else
    echo "기존 .env를 보존합니다. 필요하면 .env.robot-ready와 비교하세요."
fi

docker compose config >/dev/null
docker compose build gateway voice-command
docker compose up -d gateway voice-command
docker compose ps
docker compose logs --tail=50 gateway voice-command

echo
echo "GUI 설정"
echo "  포트: 9999"
echo "  음성 포트: 10000"
echo "  로봇 이름: raspberrypi.local"
echo "  토큰: $(sed -n 's/^COMMAND_TOKEN=//p' .env)"
