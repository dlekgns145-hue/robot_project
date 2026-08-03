#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${GUI_DIR}/.venv"

if ! command -v python3 >/dev/null 2>&1; then
    echo "Python 3.10 or newer is required." >&2
    exit 1
fi

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip
"${VENV_DIR}/bin/pip" install -r "${GUI_DIR}/requirements.txt"
chmod +x "${GUI_DIR}/Robot Control v2.app/Contents/MacOS/robot-control-v2"

echo "GUI installation complete. Double-click Robot Control v2.app."
