#!/usr/bin/env bash
set -euo pipefail

USER_GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
USER_GUI_VENV="${USER_GUI_DIR}/.venv"

if ! command -v python3 >/dev/null 2>&1; then
    echo "Python 3.10 이상이 필요합니다." >&2
    exit 1
fi

python3 -m venv "${USER_GUI_VENV}"
"${USER_GUI_VENV}/bin/python" -m pip install --upgrade pip
"${USER_GUI_VENV}/bin/pip" install -r "${USER_GUI_DIR}/requirements.txt"

echo "사용자용 GUI 설치가 완료되었습니다."
echo "다음부터는 run_user_gui_macos.command를 더블클릭하세요."
