#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ROBOT_COMPUTE_CONTAINER:-robot-v2-compute-mapping}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="${SCRIPT_DIR}/phone_camera_layer_test.py"

if [[ $# -lt 1 || $# -gt 3 ]]; then
    printf 'Usage: %s CAMERA_URL [RESULT_DIRECTORY] [NORMALIZED_CROP]\n' "$0" >&2
    exit 2
fi
camera_url="$1"
result_directory="${2:-${PWD}/phone-camera-layer-test}"
normalized_crop="${3:-}"
container_output="/tmp/phone-camera-layer-test-$(date +%s)-$$"
container_script="/tmp/phone_camera_layer_test-$$.py"

if [[ ! -f "${PYTHON_SCRIPT}" ]]; then
    printf 'Missing test program: %s\n' "${PYTHON_SCRIPT}" >&2
    exit 1
fi
if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
    printf 'Compute container is unavailable: %s\n' "${CONTAINER}" >&2
    exit 1
fi

mkdir -p "${result_directory}"
docker cp "${PYTHON_SCRIPT}" "${CONTAINER}:${container_script}" >/dev/null
cleanup() {
    docker exec "${CONTAINER}" rm -rf \
        "${container_script}" "${container_output}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

command=(
    docker exec "${CONTAINER}" python3 "${container_script}"
    --url "${camera_url}"
    --output-dir "${container_output}"
)
if [[ -n "${normalized_crop}" ]]; then
    command+=(--crop "${normalized_crop}")
fi
"${command[@]}"
docker cp "${CONTAINER}:${container_output}/." "${result_directory}/" >/dev/null
printf 'Phone camera layer test results: %s\n' "${result_directory}"
