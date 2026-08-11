#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROBOT_COMPUTE_CONTAINER:-robot-v2-compute-mapping}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/phone_scene_capture.py"
STATE_DIR="${HOME}/.local/state/robot-phone-capture"
STATE_FILE="${STATE_DIR}/active-session"
RESULT_ROOT="${HOME}/phone-captures"

usage() {
    cat <<'EOF'
Usage:
  manage_phone_scene_capture.sh start CAMERA_URL [SESSION_NAME]
  manage_phone_scene_capture.sh status
  manage_phone_scene_capture.sh stop

The phone-only session stores selected frames and timestamps. It has no metric
pose until aligned with synchronized robot odometry or LiDAR control points.
EOF
}

load_state() {
    if [[ ! -f "${STATE_FILE}" ]]; then
        printf 'No active phone capture session.\n' >&2
        exit 1
    fi
    # Values written below are restricted to safe path/name characters.
    # shellcheck disable=SC1090
    source "${STATE_FILE}"
}

action="${1:-}"
case "${action}" in
    start)
        camera_url="${2:-}"
        session_name="${3:-$(date +%Y%m%d-%H%M%S)}"
        if [[ ! "${camera_url}" =~ ^https?://[^[:space:]]+$ ]]; then
            printf 'A complete HTTP camera URL is required.\n' >&2
            exit 2
        fi
        if [[ ! "${session_name}" =~ ^[A-Za-z0-9._-]+$ ]]; then
            printf 'Session name may contain only letters, numbers, dot, dash, underscore.\n' >&2
            exit 2
        fi
        if [[ -f "${STATE_FILE}" ]]; then
            printf 'A capture state already exists; run status or stop first.\n' >&2
            exit 1
        fi
        if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
            printf 'Compute container is unavailable: %s\n' "${CONTAINER}" >&2
            exit 1
        fi
        mkdir -p "${STATE_DIR}" "${RESULT_ROOT}"
        container_script="/tmp/phone_scene_capture-${UID}.py"
        container_output="/tmp/phone-scene-${UID}-${session_name}"
        result_directory="${RESULT_ROOT}/${session_name}"
        docker cp "${PYTHON_SCRIPT}" "${CONTAINER}:${container_script}" >/dev/null
        docker exec "${CONTAINER}" rm -rf "${container_output}"
        docker exec -d \
            -e CAMERA_CAPTURE_URL="${camera_url}" \
            "${CONTAINER}" \
            python3 "${container_script}" \
            --url-env CAMERA_CAPTURE_URL \
            --output-dir "${container_output}" \
            --redact-region "${PHONE_REDACT_REGION:-0.63,0.04,0.33,0.33}"
        {
            printf 'session_name=%q\n' "${session_name}"
            printf 'container_script=%q\n' "${container_script}"
            printf 'container_output=%q\n' "${container_output}"
            printf 'result_directory=%q\n' "${result_directory}"
        } >"${STATE_FILE}"
        printf 'Phone scene capture started: %s\n' "${session_name}"
        printf 'Walk slowly, keep the rear camera aimed at surfaces, and pause at turns.\n'
        ;;
    status)
        load_state
        if docker exec "${CONTAINER}" test -f "${container_output}/progress.json"; then
            docker exec "${CONTAINER}" cat "${container_output}/progress.json"
        else
            printf 'Session is starting; progress is not available yet.\n'
        fi
        ;;
    stop)
        load_state
        docker exec "${CONTAINER}" touch "${container_output}/STOP"
        for _attempt in $(seq 1 20); do
            if docker exec "${CONTAINER}" test -f "${container_output}/FINISHED"; then
                break
            fi
            sleep 0.5
        done
        if ! docker exec "${CONTAINER}" test -f "${container_output}/FINISHED"; then
            printf 'Recorder did not finish within 10 seconds; try stop again.\n' >&2
            exit 1
        fi
        mkdir -p "${result_directory}"
        docker cp "${CONTAINER}:${container_output}/." "${result_directory}/" >/dev/null
        docker exec "${CONTAINER}" rm -rf \
            "${container_output}" "${container_script}"
        rm -f "${STATE_FILE}"
        printf 'Phone scene capture saved: %s\n' "${result_directory}"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage >&2
        exit 2
        ;;
esac
