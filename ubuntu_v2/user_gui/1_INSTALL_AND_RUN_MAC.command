#!/usr/bin/env bash
set -euo pipefail

USER_GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Robot Companion 사용자용 GUI를 설치합니다."
echo "첫 설치는 패키지 다운로드 때문에 몇 분 걸릴 수 있습니다."
bash "${USER_GUI_DIR}/setup_user_gui_macos.sh"
exec bash "${USER_GUI_DIR}/run_user_gui_macos.command"
