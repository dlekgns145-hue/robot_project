#!/usr/bin/env python3
"""Exercise the camera-to-map layer pipeline without robot hardware.

The generated occupancy grid and poses are synthetic.  This validates camera
transport, JPEG decoding, projection, material classification, and PNG/JSON
output, but it is deliberately not a navigation map.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--output-dir", default="/tmp/iphone-layer-test")
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument(
        "--crop",
        help="optional normalized x,y,width,height crop, for example 0.63,0.04,0.33,0.33",
    )
    parser.add_argument(
        "--module-dir",
        default=os.getenv(
            "ROBOT_NAVIGATION_MODULE_DIR", "/opt/robot-control/navigation"
        ),
    )
    return parser.parse_args()


def redact_url(url: str) -> str:
    return re.sub(r"(?<=//)[^/@]+@", "<credentials>@", url)


def parse_crop(value: str | None) -> tuple[float, float, float, float] | None:
    if not value:
        return None
    parts = tuple(float(part) for part in value.split(","))
    if len(parts) != 4:
        raise ValueError("crop must contain x,y,width,height")
    x, y, width, height = parts
    if min(parts) < 0.0 or width <= 0.0 or height <= 0.0:
        raise ValueError("crop values must be non-negative with positive size")
    if x + width > 1.0 or y + height > 1.0:
        raise ValueError("normalized crop must fit inside the frame")
    return x, y, width, height


def crop_frame(
    frame: np.ndarray, crop: tuple[float, float, float, float] | None
) -> np.ndarray:
    if crop is None:
        return frame
    frame_height, frame_width = frame.shape[:2]
    x, y, width, height = crop
    left = int(round(x * frame_width))
    top = int(round(y * frame_height))
    right = int(round((x + width) * frame_width))
    bottom = int(round((y + height) * frame_height))
    selected = frame[top:bottom, left:right]
    if selected.size == 0 or selected.shape[0] < 40 or selected.shape[1] < 40:
        raise ValueError("camera crop is empty or too small")
    return selected.copy()


def capture_samples(
    url: str,
    count: int,
    crop: tuple[float, float, float, float] | None,
):
    from map_texture_core import CameraSample

    if count < 1 or count > 60:
        raise ValueError("samples must be between 1 and 60")
    capture = cv2.VideoCapture(url)
    if not capture.isOpened():
        raise RuntimeError("camera stream did not open")
    samples = []
    first_raw_frame = None
    first_frame = None
    started = time.monotonic()
    try:
        while len(samples) < count and time.monotonic() - started < 15.0:
            ok, frame = capture.read()
            if not ok or frame is None or not frame.size:
                continue
            if first_raw_frame is None:
                first_raw_frame = frame.copy()
            frame = crop_frame(frame, crop)
            if first_frame is None:
                first_frame = frame.copy()
            encoded_ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82]
            )
            if not encoded_ok:
                continue
            # A short synthetic trajectory is used only to exercise projection.
            index = len(samples)
            samples.append(
                CameraSample(encoded.tobytes(), index * 0.07, 0.0, 0.0)
            )
            time.sleep(0.12)
    finally:
        capture.release()
    if first_frame is None or len(samples) < count:
        raise RuntimeError(f"received only {len(samples)} of {count} camera frames")
    return first_raw_frame, first_frame, samples


def synthetic_map():
    from map_texture_core import SavedMapInfo, world_to_map_pixel

    info = SavedMapInfo(
        width=320,
        height=240,
        resolution=0.025,
        origin_x=-1.0,
        origin_y=-3.0,
        origin_yaw=0.0,
    )
    occupancy = np.full((info.height, info.width), 254, dtype=np.uint8)
    occupancy[:8, :] = 205
    occupancy[-8:, :] = 205
    occupancy[:, :8] = 205
    occupancy[:, -8:] = 205

    # A known occupied strip intersects the camera projection so the visual-only
    # obstacle material classifier is exercised without inventing geometry from
    # the phone image.
    left, bottom = world_to_map_pixel(1.35, -1.15, info)
    right, top = world_to_map_pixel(1.65, 1.15, info)
    x0, x1 = sorted((int(left), int(right)))
    y0, y1 = sorted((int(top), int(bottom)))
    occupancy[max(0, y0) : min(info.height, y1 + 1), max(0, x0) : min(info.width, x1 + 1)] = 0
    return occupancy, info


def main() -> int:
    args = parse_args()
    sys.path.insert(0, args.module_dir)
    from map_texture_core import compose_visual_layers

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    crop = parse_crop(args.crop)
    raw_frame, frame, samples = capture_samples(args.url, args.samples, crop)
    occupancy, info = synthetic_map()
    texture, obstacles, materials = compose_visual_layers(
        occupancy,
        info,
        samples,
        near_m=0.18,
        far_m=2.0,
        near_width_m=0.85,
        far_width_m=1.8,
        source_top_fraction=0.50,
    )

    cv2.imwrite(str(output / "iphone_frame_raw.jpg"), raw_frame)
    cv2.imwrite(str(output / "iphone_frame.jpg"), frame)
    cv2.imwrite(str(output / "synthetic_map.pgm"), occupancy)
    cv2.imwrite(str(output / "iphone_texture.png"), texture)
    cv2.imwrite(str(output / "iphone_obstacles.png"), obstacles)
    (output / "synthetic_map.yaml").write_text(
        "image: synthetic_map.pgm\n"
        "resolution: 0.025\n"
        "origin: [-1.0, -3.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    (output / "iphone_materials.json").write_text(
        json.dumps(materials, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    report = {
        "camera_url": redact_url(args.url),
        "frame_width": int(frame.shape[1]),
        "frame_height": int(frame.shape[0]),
        "samples": len(samples),
        "crop": args.crop or "full-frame",
        "synthetic_map": True,
        "navigation_safe": False,
        "materials": materials,
    }
    (output / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
