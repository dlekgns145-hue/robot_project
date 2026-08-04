#!/usr/bin/env bash
set -euo pipefail

USER_GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
USER_GUI_PYTHON="${USER_GUI_DIR}/.venv/bin/python"

if [[ ! -x "${USER_GUI_PYTHON}" ]]; then
    echo "사용자용 GUI가 아직 설치되지 않았습니다."
    echo "1_INSTALL_AND_RUN_MAC.command를 먼저 실행하세요."
    read -r -p "Enter를 누르면 닫힙니다..."
    exit 1
fi

exec "${USER_GUI_PYTHON}" "${USER_GUI_DIR}/main.py"
