#!/usr/bin/env bash
set -euo pipefail

INSTALL_DIR="${INSTALL_DIR:-/opt/robot-control-server}"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run as root: sudo ./UNINSTALL_SERVER.sh" >&2
    exit 1
fi

systemctl disable --now robot-control-server.service 2>/dev/null || true
if [[ -d "${INSTALL_DIR}/ubuntu_v2" ]] && command -v docker >/dev/null 2>&1; then
    docker compose \
        --project-directory "${INSTALL_DIR}/ubuntu_v2" \
        --profile compute down || true
fi
rm -f /etc/systemd/system/robot-control-server.service
systemctl daemon-reload

echo "The service was removed. Runtime data remains in ${INSTALL_DIR}."
echo "Delete that directory manually only after backing up maps and operations data."

