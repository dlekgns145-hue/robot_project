#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
PROXY_SCRIPT="${SCRIPT_DIR}/mac_lan_proxy.py"
PROXY_RUNNER="${SCRIPT_DIR}/run_macos_lan_proxy.sh"
GUI_PYTHON="${PROJECT_DIR}/ubuntu_v2/desktop_gui/.venv/bin/python"
VM_IP="${1:-192.168.64.15}"
ALLOWED_CLIENT="${2:-}"
VM_PORT="${3:-9999}"
LISTEN_PORT="${4:-9999}"
LABEL="com.robot-project.ubuntu-lan-proxy"
DOMAIN="gui/$(id -u)"
PLIST_PATH="${HOME}/Library/LaunchAgents/${LABEL}.plist"
LOG_PATH="${HOME}/Library/Logs/robot-control-lan-proxy.log"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "This installer must run on the Mac that hosts the Ubuntu UTM VM." >&2
    exit 1
fi
if [[ -z "${ALLOWED_CLIENT}" ]]; then
    echo "Usage: $0 [VM_IP] <allowed-PC-IP-or-CIDR> [VM_PORT] [LISTEN_PORT]" >&2
    echo "Example: $0 192.168.64.15 172.30.1.123" >&2
    exit 2
fi
if [[ ! -x "${GUI_PYTHON}" ]]; then
    echo "GUI Python not found: ${GUI_PYTHON}" >&2
    echo "Run ubuntu_v2/desktop_gui/setup_gui_macos.sh first." >&2
    exit 1
fi
if ! nc -z -w 2 "${VM_IP}" "${VM_PORT}"; then
    echo "Ubuntu gateway is not reachable at ${VM_IP}:${VM_PORT}." >&2
    exit 1
fi

DEFAULT_INTERFACE="$(route -n get default | awk '/interface:/{print $2; exit}')"
LAN_IP="$(ipconfig getifaddr "${DEFAULT_INTERFACE}")"
LAN_CIDR="$("${GUI_PYTHON}" -c \
    'import ipaddress,sys; value=sys.argv[1]; print(ipaddress.ip_network(value if "/" in value else value + "/32", strict=False))' \
    "${ALLOWED_CLIENT}")"

launchctl bootout "${DOMAIN}/${LABEL}" 2>/dev/null || true
if lsof -nP -iTCP:"${LISTEN_PORT}" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "TCP port ${LISTEN_PORT} is already in use on this Mac." >&2
    lsof -nP -iTCP:"${LISTEN_PORT}" -sTCP:LISTEN >&2
    exit 1
fi

mkdir -p "${HOME}/Library/LaunchAgents" "${HOME}/Library/Logs"
"${GUI_PYTHON}" - \
    "${PLIST_PATH}" "${LABEL}" "${PROXY_RUNNER}" "${GUI_PYTHON}" \
    "${PROXY_SCRIPT}" "${VM_IP}" "${VM_PORT}" "${LISTEN_PORT}" \
    "${LAN_CIDR}" "${DEFAULT_INTERFACE}" "${LOG_PATH}" <<'PY'
import plistlib
import sys

(
    plist_path,
    label,
    proxy_runner,
    python_path,
    proxy_script,
    vm_ip,
    vm_port,
    listen_port,
    lan_cidr,
    default_interface,
    log_path,
) = sys.argv[1:]

payload = {
    "Label": label,
    "ProgramArguments": [
        proxy_runner,
        python_path,
        proxy_script,
        vm_ip,
        vm_port,
        listen_port,
        lan_cidr,
        default_interface,
    ],
    "RunAtLoad": True,
    "KeepAlive": True,
    "ProcessType": "Background",
    "ThrottleInterval": 5,
    "StandardOutPath": log_path,
    "StandardErrorPath": log_path,
}
with open(plist_path, "wb") as stream:
    plistlib.dump(payload, stream, sort_keys=False)
PY

chmod 0600 "${PLIST_PATH}"
launchctl bootstrap "${DOMAIN}" "${PLIST_PATH}"
launchctl enable "${DOMAIN}/${LABEL}"
launchctl kickstart -k "${DOMAIN}/${LABEL}"
sleep 1

if ! lsof -nP -iTCP:"${LISTEN_PORT}" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "Proxy failed to start. Log: ${LOG_PATH}" >&2
    tail -n 30 "${LOG_PATH}" >&2 || true
    exit 1
fi

echo "Ubuntu gateway is now available to the local network."
echo "  Other PC GUI server IP: ${LAN_IP}"
echo "  Port: ${LISTEN_PORT}"
echo "  Allowed client network: ${LAN_CIDR}"
echo "  Target Ubuntu VM: ${VM_IP}:${VM_PORT}"
echo "  Log: ${LOG_PATH}"
