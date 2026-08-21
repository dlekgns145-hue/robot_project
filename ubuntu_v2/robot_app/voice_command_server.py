#!/usr/bin/env python3
"""Small authenticated HTTP bridge from WAV audio to safe robot voice intents.

The speech model only produces text.  This process owns the deliberately small
allow-list that turns that text into one of the four commands understood by the
Android app.  It never talks to the robot directly.
"""

from __future__ import annotations

import hmac
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import secrets
import unicodedata
import urllib.error
import urllib.request
import wave
from array import array
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


VOICE_PORT = int(os.getenv("VOICE_PORT", "10000"))
COMMAND_TOKEN = os.getenv("COMMAND_TOKEN", "")
WHISPER_CLI = os.getenv("WHISPER_CLI", "/usr/local/bin/whisper-cli")
WHISPER_MODEL = os.getenv("WHISPER_MODEL", "/models/ggml-small.bin")
WHISPER_THREADS = max(1, int(os.getenv("WHISPER_THREADS", "4")))
WHISPER_TIMEOUT_SEC = max(5.0, float(os.getenv("WHISPER_TIMEOUT_SEC", "60")))
WHISPER_SERVER_URL = os.getenv("WHISPER_SERVER_URL", "").strip()
MAX_AUDIO_BYTES = max(64_000, int(os.getenv("VOICE_MAX_AUDIO_BYTES", "1048576")))
MIN_AUDIO_RMS = max(0.0, float(os.getenv("VOICE_MIN_AUDIO_RMS", "0.004")))

WHISPER_PROMPT = (
    "로봇 음성 명령입니다. 가능한 명령은 기다려, 멈춰, 따라와, 집으로 가입니다. "
    "사용자가 한국 지역 방언으로 말할 수도 있습니다."
)

# These are intentional, reviewed aliases rather than unrestricted fuzzy matching.
# Normalize removes spacing and punctuation before comparison.
COMMAND_ALIASES: dict[str, tuple[str, ...]] = {
    "STOP": (
        "멈춰", "멈춰라", "멈추라", "멈추세요", "정지", "정지해",
        "정지하세요", "서라", "그만", "그만해", "스톱",
        "멈추소", "멈추이소", "멈춰예", "서이소", "고마해",
        "고마해라", "멈춥서", "멈춰봅서",
    ),
    "WAIT": (
        "기다려", "기다려라", "기다리라", "기다리세요", "기다려줘",
        "기다려봐", "잠깐", "잠깐만", "잠깐기다려", "잠시기다려",
        "기달려", "기달려라", "기다리소", "기다리이소", "기다려예",
        "지달려", "지둘려", "지둘러", "기다립서", "기다려봅서",
    ),
    "FOLLOW": (
        "따라와", "따라와라", "따라오라", "따라오세요", "따라와줘",
        "따라와봐", "따라오너라", "쫓아와", "뒤따라와", "따라온나",
        "따라오소", "따라오이소", "따라와예", "따라옵서", "따라와봅서",
    ),
    "GO_HOME": (
        "집으로가", "집으로가라", "집으로가세요", "집에가", "집에가라",
        "집에가세요", "집에가줘", "집으로돌아가", "집에돌아가",
        "귀가", "귀가해", "복귀", "복귀해", "귀환", "귀환해", "홈으로가",
        "집으로가소", "집에가소", "집으로가이소", "집에가이소",
        "집으로가예", "집으로갑서", "집에갑서", "집드레가라", "집드레갑서",
    ),
}

_LEADING_FILLERS = (
    "로봇아", "로봇", "야", "이제", "지금", "어서", "얼른", "제발", "좀",
)
_TRAILING_FILLERS = ("해줘", "해주세요", "주라", "줘", "요")
_WHISPER_LOCK = threading.Lock()


def normalize_transcript(text: str) -> str:
    normalized = unicodedata.normalize("NFKC", text or "").lower()
    normalized = re.sub(r"[^0-9a-z가-힣]", "", normalized)
    changed = True
    while changed:
        changed = False
        for filler in _LEADING_FILLERS:
            if normalized.startswith(filler) and len(normalized) > len(filler):
                normalized = normalized[len(filler):]
                changed = True
                break
    for filler in _TRAILING_FILLERS:
        if normalized.endswith(filler) and len(normalized) > len(filler) + 1:
            candidate = normalized[:-len(filler)]
            if any(candidate == alias for aliases in COMMAND_ALIASES.values() for alias in aliases):
                normalized = candidate
                break
    return normalized


def command_from_transcript(text: str) -> tuple[str | None, str | None]:
    """Return only exact allow-listed commands; unknown speech is a no-op."""
    normalized = normalize_transcript(text)
    for command, aliases in COMMAND_ALIASES.items():
        if normalized in aliases:
            return command, normalized
    return None, None


def _validate_wav(path: Path) -> float:
    try:
        with wave.open(str(path), "rb") as source:
            if source.getnchannels() != 1:
                raise ValueError("audio must be mono")
            if source.getsampwidth() != 2:
                raise ValueError("audio must use 16-bit PCM")
            if source.getframerate() != 16_000:
                raise ValueError("audio sample rate must be 16000 Hz")
            frame_count = source.getnframes()
            if frame_count < 8_000 or frame_count > 16_000 * 8:
                raise ValueError("audio duration must be between 0.5 and 8 seconds")
            samples = array("h", source.readframes(frame_count))
    except (EOFError, wave.Error) as error:
        raise ValueError("invalid WAV audio") from error
    if not samples:
        raise ValueError("audio is empty")
    square_mean = sum(sample * sample for sample in samples) / len(samples)
    return math.sqrt(square_mean) / 32768.0


def transcribe_wav(wav_path: Path, output_dir: Path) -> str:
    if WHISPER_SERVER_URL:
        return _transcribe_with_server(wav_path)

    cli = Path(WHISPER_CLI)
    model = Path(WHISPER_MODEL)
    if not cli.is_file():
        raise RuntimeError(f"whisper-cli not found: {cli}")
    if not model.is_file():
        raise RuntimeError(f"Whisper model not found: {model}")

    output_base = output_dir / "transcript"
    command = [
        str(cli), "-m", str(model), "-f", str(wav_path), "-l", "ko",
        "-t", str(WHISPER_THREADS), "-nt", "-np", "-otxt", "-of",
        str(output_base), "--prompt", WHISPER_PROMPT,
    ]
    with _WHISPER_LOCK:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=WHISPER_TIMEOUT_SEC,
            check=False,
        )
    if completed.returncode != 0:
        detail = completed.stderr.strip().splitlines()
        message = detail[-1] if detail else "unknown whisper-cli failure"
        raise RuntimeError(f"speech recognition failed: {message[:240]}")
    transcript_file = output_base.with_suffix(".txt")
    if not transcript_file.is_file():
        raise RuntimeError("whisper-cli did not create a transcript")
    return " ".join(transcript_file.read_text(encoding="utf-8").split()).strip()


def _transcribe_with_server(wav_path: Path) -> str:
    boundary = "robot-voice-" + secrets.token_hex(12)
    body = bytearray()

    def add_field(name: str, value: str) -> None:
        body.extend(f"--{boundary}\r\n".encode())
        body.extend(
            f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode()
        )
        body.extend(value.encode("utf-8"))
        body.extend(b"\r\n")

    add_field("response_format", "json")
    add_field("temperature", "0.0")
    add_field("no_speech_thold", "0.8")
    audio = wav_path.read_bytes()
    body.extend(f"--{boundary}\r\n".encode())
    body.extend(
        b'Content-Disposition: form-data; name="file"; filename="command.wav"\r\n'
    )
    body.extend(b"Content-Type: audio/wav\r\n\r\n")
    body.extend(audio)
    body.extend(b"\r\n")
    body.extend(f"--{boundary}--\r\n".encode())

    request = urllib.request.Request(
        WHISPER_SERVER_URL,
        data=bytes(body),
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "Accept": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=WHISPER_TIMEOUT_SEC) as response:
            payload = json.load(response)
    except (urllib.error.URLError, json.JSONDecodeError) as error:
        raise RuntimeError("persistent whisper-server request failed") from error
    transcript = payload.get("text") if isinstance(payload, dict) else None
    if not isinstance(transcript, str):
        raise RuntimeError("persistent whisper-server returned no transcript")
    return " ".join(transcript.split()).strip()


class VoiceRequestHandler(BaseHTTPRequestHandler):
    server_version = "RobotVoice/1.0"

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/health":
            self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        self._json(HTTPStatus.OK, {
            "ok": True,
            "model_ready": Path(WHISPER_MODEL).is_file(),
        })

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/v1/voice/command":
            self._json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        supplied_token = self.headers.get("X-Command-Token", "")
        if not COMMAND_TOKEN or not hmac.compare_digest(supplied_token, COMMAND_TOKEN):
            self._json(HTTPStatus.UNAUTHORIZED, {
                "ok": False, "error": "invalid command token",
            })
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > MAX_AUDIO_BYTES:
            self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {
                "ok": False, "error": "invalid audio size",
            })
            return
        audio = self.rfile.read(length)
        if len(audio) != length:
            self._json(HTTPStatus.BAD_REQUEST, {
                "ok": False, "error": "incomplete audio body",
            })
            return

        try:
            with tempfile.TemporaryDirectory(prefix="robot-voice-") as temporary:
                directory = Path(temporary)
                wav_path = directory / "command.wav"
                wav_path.write_bytes(audio)
                rms = _validate_wav(wav_path)
                if rms < MIN_AUDIO_RMS:
                    self._json(HTTPStatus.OK, {
                        "ok": True, "transcript": "", "command": None,
                        "accepted": False, "reason": "silence",
                    })
                    return
                transcript = transcribe_wav(wav_path, directory)
                command, matched_phrase = command_from_transcript(transcript)
        except ValueError as error:
            self._json(HTTPStatus.UNPROCESSABLE_ENTITY, {
                "ok": False, "error": str(error),
            })
            return
        except subprocess.TimeoutExpired:
            self._json(HTTPStatus.GATEWAY_TIMEOUT, {
                "ok": False, "error": "speech recognition timed out",
            })
            return
        except RuntimeError as error:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {
                "ok": False, "error": str(error),
            })
            return

        self._json(HTTPStatus.OK, {
            "ok": True,
            "transcript": transcript,
            "command": command,
            "accepted": command is not None,
            "matched_phrase": matched_phrase,
            "reason": None if command is not None else "not_allowed",
        })

    def log_message(self, format_string: str, *args: Any) -> None:
        # Do not log transcripts or the authentication token.
        print(f"voice-api {self.client_address[0]} {format_string % args}", flush=True)

    def _json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)


def main() -> None:
    server = ThreadingHTTPServer(("0.0.0.0", VOICE_PORT), VoiceRequestHandler)
    print(
        f"voice command server listening on :{VOICE_PORT}; model={WHISPER_MODEL}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
