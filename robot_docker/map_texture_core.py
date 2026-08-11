"""Pure camera-to-map projection helpers used by the mapping runtime.

This module intentionally has no ROS dependency so the projection and file
format behavior can be tested on the desktop before deploying to the robot.
"""

from __future__ import annotations

import ast
import json
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class SavedMapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


@dataclass(frozen=True)
class CameraSample:
    jpeg: bytes
    x: float
    y: float
    yaw: float


def world_to_map_pixel(x: float, y: float, info: SavedMapInfo) -> tuple[float, float]:
    dx = x - info.origin_x
    dy = y - info.origin_y
    cosine = math.cos(info.origin_yaw)
    sine = math.sin(info.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        local_x / info.resolution - 0.5,
        info.height - local_y / info.resolution - 0.5,
    )


def _world_ground_point(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    forward: float,
    left: float,
) -> tuple[float, float]:
    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    return (
        robot_x + cosine * forward - sine * left,
        robot_y + sine * forward + cosine * left,
    )


def load_saved_map(map_output: str) -> tuple[np.ndarray, SavedMapInfo]:
    image = cv2.imread(f"{map_output}.pgm", cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"saved occupancy image is unavailable: {map_output}.pgm")
    metadata: dict[str, str] = {}
    with open(f"{map_output}.yaml", "r", encoding="utf-8") as yaml_file:
        for raw_line in yaml_file:
            line = raw_line.split("#", 1)[0].strip()
            if line and ":" in line:
                key, value = (part.strip() for part in line.split(":", 1))
                metadata[key] = value
    origin = ast.literal_eval(metadata.get("origin", "[0, 0, 0]"))
    if not isinstance(origin, (list, tuple)) or len(origin) < 3:
        raise ValueError("saved map origin must contain x, y, and yaw")
    resolution = float(metadata["resolution"])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("saved map resolution must be positive")
    height, width = image.shape
    return image, SavedMapInfo(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
    )


def _validate_projection(
    *,
    near_m: float,
    far_m: float,
    near_width_m: float,
    far_width_m: float,
    source_top_fraction: float,
) -> None:
    values = (near_m, far_m, near_width_m, far_width_m, source_top_fraction)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("camera projection values must be finite")
    if near_m < 0.0 or far_m <= near_m:
        raise ValueError("projection_far_m must be greater than projection_near_m")
    if near_width_m <= 0.0 or far_width_m <= 0.0:
        raise ValueError("camera projection widths must be positive")
    if not 0.0 < source_top_fraction < 0.98:
        raise ValueError("projection_source_top_fraction must be between 0 and 0.98")


def compose_texture(
    occupancy: np.ndarray,
    info: SavedMapInfo,
    samples: list[CameraSample],
    *,
    near_m: float = 0.25,
    far_m: float = 3.5,
    near_width_m: float = 1.25,
    far_width_m: float = 2.6,
    source_top_fraction: float = 0.42,
) -> np.ndarray:
    """Blend camera ground trapezoids into occupancy-map pixel coordinates."""
    texture, _obstacles, _materials = compose_visual_layers(
        occupancy,
        info,
        samples,
        near_m=near_m,
        far_m=far_m,
        near_width_m=near_width_m,
        far_width_m=far_width_m,
        source_top_fraction=source_top_fraction,
    )
    return texture


def _project_camera_mosaic(
    info: SavedMapInfo,
    samples: list[CameraSample],
    *,
    near_m: float,
    far_m: float,
    near_width_m: float,
    far_width_m: float,
    source_top_fraction: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return averaged BGR camera pixels and their accumulated confidence."""
    canvas_sum = np.zeros((info.height, info.width, 3), dtype=np.float32)
    canvas_weight = np.zeros((info.height, info.width), dtype=np.float32)

    for sample in samples:
        encoded = np.frombuffer(sample.jpeg, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None or frame.size == 0:
            continue
        frame_height, frame_width = frame.shape[:2]
        source = np.float32(
            [
                [frame_width * 0.27, frame_height * source_top_fraction],
                [frame_width * 0.73, frame_height * source_top_fraction],
                [frame_width * 0.98, frame_height * 0.98],
                [frame_width * 0.02, frame_height * 0.98],
            ]
        )
        ground_corners = [
            (far_m, far_width_m / 2.0),
            (far_m, -far_width_m / 2.0),
            (near_m, -near_width_m / 2.0),
            (near_m, near_width_m / 2.0),
        ]
        destination = np.float32(
            [
                world_to_map_pixel(
                    *_world_ground_point(
                        sample.x, sample.y, sample.yaw, forward, left
                    ),
                    info,
                )
                for forward, left in ground_corners
            ]
        )
        left = max(0, int(math.floor(float(destination[:, 0].min()))) - 2)
        top = max(0, int(math.floor(float(destination[:, 1].min()))) - 2)
        right = min(
            info.width, int(math.ceil(float(destination[:, 0].max()))) + 3
        )
        bottom = min(
            info.height, int(math.ceil(float(destination[:, 1].max()))) + 3
        )
        if right <= left or bottom <= top:
            continue
        local_destination = destination - np.float32([left, top])
        homography = cv2.getPerspectiveTransform(source, local_destination)
        source_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
        cv2.fillConvexPoly(source_mask, source.astype(np.int32), 255)
        warped = cv2.warpPerspective(
            frame, homography, (right - left, bottom - top), flags=cv2.INTER_LINEAR
        )
        weight = cv2.warpPerspective(
            source_mask,
            homography,
            (right - left, bottom - top),
            flags=cv2.INTER_LINEAR,
        ).astype(np.float32) / 255.0
        weight = cv2.GaussianBlur(weight, (0, 0), sigmaX=1.2)
        canvas_sum[top:bottom, left:right] += (
            warped.astype(np.float32) * weight[..., None]
        )
        canvas_weight[top:bottom, left:right] += weight

    averaged = canvas_sum / np.maximum(canvas_weight[..., None], 0.001)
    return np.clip(averaged, 0, 255).astype(np.uint8), canvas_weight


def _material_summary(camera_bgr: np.ndarray, obstacle_mask: np.ndarray) -> dict:
    counts = {
        "vegetation": 0,
        "earth_or_wood": 0,
        "stone_or_concrete": 0,
        "reflective_or_synthetic": 0,
        "ambiguous_neutral": 0,
        "other": 0,
    }
    observed_pixels = int(np.count_nonzero(obstacle_mask))
    if observed_pixels == 0:
        return {
            "method": "camera-color-heuristic-v2",
            "observed_pixels": 0,
            "counts": counts,
            "dominant": "unknown",
            "dominant_share": 0.0,
            "confidence": "low",
        }

    hsv = cv2.cvtColor(camera_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[..., 0]
    saturation = hsv[..., 1]
    value = hsv[..., 2]
    unclassified = obstacle_mask.copy()

    vegetation = (
        unclassified
        & (hue >= 25)
        & (hue <= 95)
        & (saturation >= 45)
        & (value >= 25)
    )
    counts["vegetation"] = int(np.count_nonzero(vegetation))
    unclassified &= ~vegetation

    earth_or_wood = (
        unclassified
        & (hue <= 30)
        & (saturation >= 35)
        & (value >= 25)
        & (value <= 230)
    )
    counts["earth_or_wood"] = int(np.count_nonzero(earth_or_wood))
    unclassified &= ~earth_or_wood

    reflective = unclassified & (saturation < 45) & (value > 215)
    counts["reflective_or_synthetic"] = int(np.count_nonzero(reflective))
    unclassified &= ~reflective

    # Very low-saturation pixels cannot reliably separate pale timber, painted
    # metal, concrete, and stone using color alone.  Keep them visibly textured
    # but label them as ambiguous instead of making a confident concrete claim.
    neutral = (
        unclassified
        & (saturation < 25)
        & (value >= 35)
        & (value <= 215)
    )
    counts["ambiguous_neutral"] = int(np.count_nonzero(neutral))
    unclassified &= ~neutral

    stone = unclassified & (saturation < 55) & (value >= 35) & (value <= 215)
    counts["stone_or_concrete"] = int(np.count_nonzero(stone))
    unclassified &= ~stone
    counts["other"] = int(np.count_nonzero(unclassified))

    dominant = max(counts, key=counts.get)
    dominant_share = counts[dominant] / observed_pixels
    if dominant_share >= 0.7:
        confidence = "high"
    elif dominant_share >= 0.5:
        confidence = "medium"
    else:
        confidence = "low"
    return {
        "method": "camera-color-heuristic-v2",
        "observed_pixels": observed_pixels,
        "counts": counts,
        "dominant": dominant if dominant_share >= 0.5 else "mixed",
        "dominant_share": round(dominant_share, 4),
        "confidence": confidence,
    }


def summarize_obstacle_materials(
    camera_bgr: np.ndarray, obstacle_mask: np.ndarray
) -> dict:
    """Public visual-only material summary for LiDAR-confirmed obstacle cells."""
    if camera_bgr.ndim != 3 or camera_bgr.shape[2] != 3:
        raise ValueError("camera obstacle image must be BGR")
    if obstacle_mask.shape != camera_bgr.shape[:2]:
        raise ValueError("obstacle mask dimensions do not match camera image")
    return _material_summary(camera_bgr, obstacle_mask.astype(bool, copy=False))


def compose_visual_layers(
    occupancy: np.ndarray,
    info: SavedMapInfo,
    samples: list[CameraSample],
    *,
    near_m: float = 0.25,
    far_m: float = 3.5,
    near_width_m: float = 1.25,
    far_width_m: float = 2.6,
    source_top_fraction: float = 0.42,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Build free-space texture, obstacle appearance, and material metadata.

    The returned obstacle layer is visual-only BGRA. Occupancy geometry remains
    in the original PGM and is never derived from the camera classification.
    """
    if occupancy.ndim != 2 or occupancy.shape != (info.height, info.width):
        raise ValueError("occupancy image dimensions do not match saved map metadata")
    _validate_projection(
        near_m=near_m,
        far_m=far_m,
        near_width_m=near_width_m,
        far_width_m=far_width_m,
        source_top_fraction=source_top_fraction,
    )
    averaged, canvas_weight = _project_camera_mosaic(
        info,
        samples,
        near_m=near_m,
        far_m=far_m,
        near_width_m=near_width_m,
        far_width_m=far_width_m,
        source_top_fraction=source_top_fraction,
    )

    base = cv2.cvtColor(occupancy, cv2.COLOR_GRAY2BGR)
    observed = canvas_weight > 0.08
    # The camera is a visual layer only. Unknown and occupied cells remain
    # unchanged here and are emphasized again by the GUI occupancy overlay.
    usable = observed & (occupancy >= 250)
    if np.any(usable):
        base[usable] = averaged[usable]

    observed_obstacles = observed & (occupancy < 65)
    obstacle_layer = np.zeros((info.height, info.width, 4), dtype=np.uint8)
    obstacle_layer[observed_obstacles, :3] = averaged[observed_obstacles]
    obstacle_layer[observed_obstacles, 3] = 225
    materials = _material_summary(averaged, observed_obstacles)
    return base, obstacle_layer, materials


def save_image_atomic(path: str, image: np.ndarray, *, extension: str | None = None) -> None:
    image_extension = extension or os.path.splitext(path)[1] or ".png"
    if image_extension.lower() not in {".png", ".pgm", ".jpg", ".jpeg"}:
        raise ValueError(f"unsupported map image extension: {image_extension}")
    ok, encoded = cv2.imencode(image_extension, image)
    if not ok:
        raise RuntimeError(f"map image encoding failed: {image_extension}")
    temporary = f"{path}.tmp"
    with open(temporary, "wb") as output_file:
        output_file.write(encoded.tobytes())
    os.replace(temporary, path)


def save_texture_atomic(path: str, image: np.ndarray) -> None:
    save_image_atomic(path, image, extension=".png")


def save_json_atomic(path: str, payload: dict) -> None:
    temporary = f"{path}.tmp"
    with open(temporary, "w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, ensure_ascii=False, separators=(",", ":"))
    os.replace(temporary, path)
