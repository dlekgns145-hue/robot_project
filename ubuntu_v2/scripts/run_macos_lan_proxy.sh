#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 7 ]]; then
    echo "usage: $0 PYTHON PROXY VM_IP VM_PORT LISTEN_PORT ALLOWED_CIDR INTERFACE" >&2
    exit 2
fi

PYTHON_PATH="$1"
PROXY_SCRIPT="$2"
VM_IP="$3"
VM_PORT="$4"
LISTEN_PORT="$5"
ALLOWED_CIDR="$6"
PREFERRED_INTERFACE="$7"

DEFAULT_INTERFACE="$(route -n get default | awk '/interface:/{print $2; exit}')"
INTERFACE="${DEFAULT_INTERFACE:-${PREFERRED_INTERFACE}}"
LAN_IP="$(ipconfig getifaddr "${INTERFACE}")"

exec "${PYTHON_PATH}" "${PROXY_SCRIPT}" \
    --listen-host "${LAN_IP}" \
    --listen-port "${LISTEN_PORT}" \
    --target-host "${VM_IP}" \
    --target-port "${VM_PORT}" \
    --allow-cidr "${ALLOWED_CIDR}"
