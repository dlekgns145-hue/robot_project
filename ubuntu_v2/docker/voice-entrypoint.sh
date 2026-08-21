#!/usr/bin/env bash
set -euo pipefail

model_path="${WHISPER_MODEL:-/models/ggml-small.bin}"
model_url="${WHISPER_MODEL_URL:-https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.bin?download=true}"
model_sha1="${WHISPER_MODEL_SHA1:-55356645c2b361a969dfd0ef2c5a50d530afd8d5}"

if [[ ! -s "${model_path}" ]]; then
    mkdir -p "$(dirname "${model_path}")"
    temporary_path="${model_path}.download"
    rm -f "${temporary_path}"
    echo "Downloading multilingual Whisper small model (about 466 MiB)..." >&2
    curl -L --fail --retry 3 --output "${temporary_path}" "${model_url}"
    actual_sha1="$(sha1sum "${temporary_path}" | cut -d ' ' -f 1)"
    if [[ "${actual_sha1}" != "${model_sha1}" ]]; then
        rm -f "${temporary_path}"
        echo "Whisper model checksum mismatch" >&2
        exit 1
    fi
    mv "${temporary_path}" "${model_path}"
fi

backend_port="${WHISPER_SERVER_PORT:-10001}"
whisper-server \
    --model "${model_path}" \
    --host 127.0.0.1 \
    --port "${backend_port}" \
    --language ko \
    --threads "${WHISPER_THREADS:-4}" \
    --no-timestamps \
    --convert \
    --prompt "로봇 음성 명령입니다. 가능한 명령은 기다려, 멈춰, 따라와, 집으로 가입니다. 사용자가 한국 지역 방언으로 말할 수도 있습니다." &
backend_pid="$!"
trap 'kill "${backend_pid}" 2>/dev/null || true; wait "${backend_pid}" 2>/dev/null || true' EXIT INT TERM

for attempt in $(seq 1 120); do
    if curl -sS --max-time 1 "http://127.0.0.1:${backend_port}/" >/dev/null 2>&1; then
        break
    fi
    if ! kill -0 "${backend_pid}" 2>/dev/null; then
        echo "whisper-server stopped during startup" >&2
        exit 1
    fi
    sleep 1
done
if ! curl -sS --max-time 2 "http://127.0.0.1:${backend_port}/" >/dev/null; then
    echo "whisper-server did not become ready" >&2
    exit 1
fi

export WHISPER_SERVER_URL="http://127.0.0.1:${backend_port}/inference"
exec python3 /opt/robot-control-v2/robot_app/voice_command_server.py
