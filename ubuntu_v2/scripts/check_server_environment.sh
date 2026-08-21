#!/usr/bin/env bash
set -u

INSTALL_DIR="${ROBOT_SERVER_DIR:-/opt/robot-control-server}"
COMPOSE_DIR="${INSTALL_DIR}/ubuntu_v2"
REQUIRED_CONTAINERS=(
    robot-v2-gateway
    robot-v2-map-postprocessor
    robot-v2-voice-command
)
failures=0

section() {
    printf '\n== %s ==\n' "$1"
}

check_equal() {
    local label="$1"
    local actual="$2"
    local expected="$3"
    if [[ "${actual}" == "${expected}" ]]; then
        printf 'OK   %-28s %s\n' "${label}" "${actual}"
    else
        printf 'FAIL %-28s expected=%s actual=%s\n' \
            "${label}" "${expected}" "${actual:-missing}"
        failures=$((failures + 1))
    fi
}

container_env() {
    local container="$1"
    local key="$2"
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${container}" \
        2>/dev/null | sed -n "s/^${key}=//p" | tail -n 1
}

section "host"
printf 'hostname: %s\n' "$(hostname)"
printf 'os:       %s\n' "$(. /etc/os-release && printf '%s' "${PRETTY_NAME}")"
printf 'arch:     %s\n' "$(uname -m)"
printf 'ip:       %s\n' "$(hostname -I 2>/dev/null | xargs)"
free -h | sed -n '1,3p'
df -h / | sed -n '1,2p'

section "automatic start"
check_equal "docker enabled" "$(systemctl is-enabled docker.service 2>/dev/null)" "enabled"
check_equal "docker active" "$(systemctl is-active docker.service 2>/dev/null)" "active"
check_equal \
    "robot server enabled" \
    "$(systemctl is-enabled robot-control-server.service 2>/dev/null)" \
    "enabled"
check_equal \
    "robot server active" \
    "$(systemctl is-active robot-control-server.service 2>/dev/null)" \
    "active"

section "containers"
if ! docker info >/dev/null 2>&1; then
    printf 'FAIL Docker API is unavailable for %s\n' "$(whoami)"
    printf '     Add the user to the docker group, then log in again.\n'
    failures=$((failures + 1))
else
    for container in "${REQUIRED_CONTAINERS[@]}"; do
        state="$(docker inspect -f '{{.State.Status}}' "${container}" 2>/dev/null || true)"
        check_equal "${container}" "${state}" "running"
    done
    docker ps --format 'table {{.Names}}\t{{.Status}}' \
        --filter name=robot-v2-
fi

section "configured endpoints"
robot_ip="$(container_env robot-v2-gateway ROBOT_IP)"
printf 'robot IP:  %s\n' "${robot_ip:-not configured}"
if [[ -n "${robot_ip}" ]]; then
    printf 'route:     '
    ip route get "${robot_ip}" 2>&1 | head -n 1
fi

voice_port="$(container_env robot-v2-voice-command VOICE_PORT)"
printf 'voice API: http://%s:%s/health\n' "$(hostname -I 2>/dev/null | awk '{print $1}')" "${voice_port:-10000}"

section "map post-processing"
map_root="${COMPOSE_DIR}/maps"
install -d "${map_root}/postprocess-inbox" "${map_root}/raw" "${map_root}/corrected"
printf 'map root:  %s\n' "${map_root}"
printf 'queued:    %s\n' "$(find "${map_root}/postprocess-inbox" -maxdepth 1 -name '*.json' | wc -l)"
if [[ -f "${map_root}/orchard_map_postprocess.json" ]]; then
    printf 'READY corrected orchard_map and report exist\n'
else
    printf 'WAIT no completed robot map has been post-processed yet\n'
fi

section "result"
if (( failures > 0 )); then
    printf 'PRECHECK FAILED: %d critical check(s) failed.\n' "${failures}"
    exit 1
fi
printf 'PRECHECK OK: gateway, voice model, and map post-processor are ready.\n'
printf 'A map WAIT line is expected until the next completed mapping run.\n'
