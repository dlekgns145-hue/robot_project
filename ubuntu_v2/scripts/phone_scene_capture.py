#!/usr/bin/env python3
"""Record a spatially unregistered phone image session for later map alignment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


STOP_REQUESTED = False


def request_stop(_signum=None, _frame=None) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_json_atomic(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def redact_url(url: str) -> str:
    return re.sub(r"(?<=//)[^/@]+@", "<credentials>@", url)


def normalized_region(value: str | None):
    if not value:
        return None
    parts = tuple(float(part) for part in value.split(","))
    if len(parts) != 4:
        raise ValueError("redact region must be x,y,width,height")
    x, y, width, height = parts
    if min(parts) < 0.0 or width <= 0.0 or height <= 0.0:
        raise ValueError("redact region values are invalid")
    if x + width > 1.0 or y + height > 1.0:
        raise ValueError("redact region must fit inside the frame")
    return x, y, width, height


def redact_region(frame: np.ndarray, region) -> None:
    if region is None:
        return
    height, width = frame.shape[:2]
    x, y, region_width, region_height = region
    left = int(round(x * width))
    top = int(round(y * height))
    right = int(round((x + region_width) * width))
    bottom = int(round((y + region_height) * height))
    frame[top:bottom, left:right] = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url")
    parser.add_argument("--url-env", default="CAMERA_CAPTURE_URL")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--maximum-frames", type=int, default=1200)
    parser.add_argument("--minimum-change", type=float, default=7.0)
    parser.add_argument("--force-interval", type=float, default=3.0)
    parser.add_argument("--minimum-sharpness", type=float, default=25.0)
    parser.add_argument("--redact-region")
    args = parser.parse_args()
    if not args.url:
        args.url = os.getenv(args.url_env, "")
    if not args.url:
        parser.error("camera URL is missing")
    if args.sample_period < 0.1 or args.sample_period > 10.0:
        parser.error("sample period must be between 0.1 and 10 seconds")
    if args.maximum_frames < 1 or args.maximum_frames > 10000:
        parser.error("maximum frames must be between 1 and 10000")
    return args


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    output = Path(args.output_dir)
    frames_dir = output / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    stop_file = output / "STOP"
    running_file = output / "RUNNING"
    finished_file = output / "FINISHED"
    running_file.write_text(utc_now() + "\n", encoding="utf-8")
    finished_file.unlink(missing_ok=True)
    region = normalized_region(args.redact_region)

    started_at = utc_now()
    started_monotonic = time.monotonic()
    previous_small = None
    last_saved_at = 0.0
    frames: list[dict] = []
    received = 0
    skipped_similar = 0
    skipped_blurry = 0
    reconnects = 0
    source_size = None
    next_sample_at = 0.0

    def status(state: str) -> dict:
        return {
            "state": state,
            "started_at": started_at,
            "updated_at": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_monotonic, 2),
            "received_frames": received,
            "saved_frames": len(frames),
            "skipped_similar": skipped_similar,
            "skipped_blurry": skipped_blurry,
            "reconnects": reconnects,
            "source_size": source_size,
        }

    try:
        while not STOP_REQUESTED and not stop_file.exists():
            capture = cv2.VideoCapture(args.url)
            if not capture.isOpened():
                capture.release()
                reconnects += 1
                write_json_atomic(output / "progress.json", status("reconnecting"))
                time.sleep(1.0)
                continue
            while not STOP_REQUESTED and not stop_file.exists():
                ok, frame = capture.read()
                if not ok or frame is None or not frame.size:
                    break
                received += 1
                now = time.monotonic()
                if now < next_sample_at:
                    continue
                next_sample_at = now + args.sample_period
                if source_size is None:
                    source_size = [int(frame.shape[1]), int(frame.shape[0])]
                frame = frame.copy()
                redact_region(frame, region)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                small = cv2.resize(gray, (160, 120), interpolation=cv2.INTER_AREA)
                sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
                brightness = float(gray.mean())
                change = (
                    255.0
                    if previous_small is None
                    else float(
                        np.mean(
                            cv2.absdiff(previous_small, small).astype(np.float32)
                        )
                    )
                )
                forced = not frames or now - last_saved_at >= args.force_interval
                if sharpness < args.minimum_sharpness and not forced:
                    skipped_blurry += 1
                    continue
                if change < args.minimum_change and not forced:
                    skipped_similar += 1
                    continue
                ok_encoded, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 86]
                )
                if not ok_encoded:
                    continue
                index = len(frames) + 1
                filename = f"frame_{index:06d}.jpg"
                data = encoded.tobytes()
                (frames_dir / filename).write_bytes(data)
                frames.append(
                    {
                        "index": index,
                        "file": f"frames/{filename}",
                        "captured_at": utc_now(),
                        "elapsed_seconds": round(now - started_monotonic, 3),
                        "width": int(frame.shape[1]),
                        "height": int(frame.shape[0]),
                        "sharpness": round(sharpness, 2),
                        "brightness": round(brightness, 2),
                        "change_score": round(change, 2),
                        "sha256": hashlib.sha256(data).hexdigest(),
                        "pose": None,
                    }
                )
                previous_small = small
                last_saved_at = now
                write_json_atomic(output / "progress.json", status("recording"))
                if len(frames) >= args.maximum_frames:
                    request_stop()
                    break
            capture.release()
            if not STOP_REQUESTED and not stop_file.exists():
                reconnects += 1
                time.sleep(0.5)
    finally:
        manifest = status("finished")
        manifest.update(
            {
                "schema": "robot-phone-scene-session-v1",
                "camera_url": redact_url(args.url),
                "sample_period_seconds": args.sample_period,
                "redacted_region": args.redact_region,
                "frames": frames,
                "spatial_registration": {
                    "status": "pending",
                    "pose_source": "unavailable_phone_only",
                    "navigation_safe": False,
                    "required_next": "align frames to synchronized robot poses or LiDAR control points",
                },
            }
        )
        write_json_atomic(output / "manifest.json", manifest)
        write_json_atomic(output / "progress.json", status("finished"))
        running_file.unlink(missing_ok=True)
        stop_file.unlink(missing_ok=True)
        finished_file.write_text(utc_now() + "\n", encoding="utf-8")
    print(json.dumps(status("finished"), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
