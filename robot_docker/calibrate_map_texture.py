#!/usr/bin/env python3
"""Build one camera-to-map preview without starting ROS.

Use a saved occupancy map, one camera frame, and the robot pose at which that
frame was captured. The output uses the same projection code as the live
mapping runtime, making it suitable for tuning the Docker ``.env`` values.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

from map_texture_core import (
    CameraSample,
    compose_texture,
    load_saved_map,
    save_texture_atomic,
)


def normalize_map_output(value: str) -> str:
    path = Path(value).expanduser()
    if path.suffix.lower() in {".pgm", ".yaml", ".yml"}:
        path = path.with_suffix("")
    return str(path)


def load_calibration_frame(source: str) -> np.ndarray:
    if "://" in source:
        capture = cv2.VideoCapture(source)
        try:
            ok, frame = capture.read()
        finally:
            capture.release()
        if not ok or frame is None or frame.size == 0:
            raise ValueError(f"camera calibration stream is unavailable: {source}")
        return frame
    frame = cv2.imread(str(Path(source).expanduser()), cv2.IMREAD_COLOR)
    if frame is None or frame.size == 0:
        raise ValueError(f"camera calibration image is unavailable: {source}")
    return frame


def build_calibration_preview(
    *,
    map_output: str,
    image_path: str,
    output_path: str,
    x: float,
    y: float,
    yaw: float,
    near_m: float,
    far_m: float,
    near_width_m: float,
    far_width_m: float,
    source_top_fraction: float,
) -> int:
    occupancy, info = load_saved_map(normalize_map_output(map_output))
    frame = load_calibration_frame(image_path)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise RuntimeError("camera calibration image could not be encoded")
    texture = compose_texture(
        occupancy,
        info,
        [CameraSample(encoded.tobytes(), float(x), float(y), float(yaw))],
        near_m=near_m,
        far_m=far_m,
        near_width_m=near_width_m,
        far_width_m=far_width_m,
        source_top_fraction=source_top_fraction,
    )
    base = cv2.cvtColor(occupancy, cv2.COLOR_GRAY2BGR)
    changed = np.any(texture != base, axis=2) & (occupancy >= 250)
    projected_pixels = int(np.count_nonzero(changed))
    if projected_pixels == 0:
        raise ValueError(
            "camera projection does not overlap a known-free part of the map; "
            "check the pose and projection distances"
        )
    save_texture_atomic(str(Path(output_path).expanduser()), texture)
    return projected_pixels


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview camera projection on a saved occupancy map"
    )
    parser.add_argument(
        "--map",
        required=True,
        help="saved map prefix or its .pgm/.yaml path",
    )
    parser.add_argument(
        "--image", required=True, help="camera still path or MJPEG stream URL"
    )
    parser.add_argument("--x", required=True, type=float, help="robot map X in meters")
    parser.add_argument("--y", required=True, type=float, help="robot map Y in meters")
    parser.add_argument(
        "--yaw-deg", required=True, type=float, help="robot map yaw in degrees"
    )
    parser.add_argument("--near", type=float, default=0.18)
    parser.add_argument("--far", type=float, default=2.0)
    parser.add_argument("--near-width", type=float, default=0.85)
    parser.add_argument("--far-width", type=float, default=1.8)
    parser.add_argument("--source-top", type=float, default=0.50)
    parser.add_argument(
        "--output",
        default="map_texture_calibration.png",
        help="preview PNG path",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    projected_pixels = build_calibration_preview(
        map_output=args.map,
        image_path=args.image,
        output_path=args.output,
        x=args.x,
        y=args.y,
        yaw=math.radians(args.yaw_deg),
        near_m=args.near,
        far_m=args.far,
        near_width_m=args.near_width,
        far_width_m=args.far_width,
        source_top_fraction=args.source_top,
    )
    print(f"preview saved: {Path(args.output).expanduser()}")
    print(f"projected known-free pixels: {projected_pixels}")
    print("copy the calibrated values to robot_docker/.env:")
    print(f"ROBOT_TEXTURE_SOURCE_TOP_FRACTION={args.source_top}")
    print(f"ROBOT_TEXTURE_NEAR_M={args.near}")
    print(f"ROBOT_TEXTURE_FAR_M={args.far}")
    print(f"ROBOT_TEXTURE_NEAR_WIDTH_M={args.near_width}")
    print(f"ROBOT_TEXTURE_FAR_WIDTH_M={args.far_width}")


if __name__ == "__main__":
    main()
