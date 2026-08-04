#!/usr/bin/env bash
set -euo pipefail

LABEL="com.robot-project.ubuntu-lan-proxy"
DOMAIN="gui/$(id -u)"
PLIST_PATH="${HOME}/Library/LaunchAgents/${LABEL}.plist"

launchctl bootout "${DOMAIN}/${LABEL}" 2>/dev/null || true
if [[ -f "${PLIST_PATH}" ]]; then
    mv "${PLIST_PATH}" "${HOME}/.Trash/${LABEL}.plist"
fi
echo "macOS LAN proxy stopped and its LaunchAgent was moved to Trash."
