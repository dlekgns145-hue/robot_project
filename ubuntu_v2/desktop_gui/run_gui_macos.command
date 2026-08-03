#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GUI_PYTHON="${GUI_DIR}/.venv/bin/python"

if [[ ! -x "${GUI_PYTHON}" ]]; then
    echo "GUI environment is not installed. Run setup_gui_macos.sh first." >&2
    read -r -p "Press Enter to close..."
    exit 1
fi

exec "${GUI_PYTHON}" "${GUI_DIR}/main.py"
