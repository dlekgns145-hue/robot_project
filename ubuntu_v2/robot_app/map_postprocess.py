#!/usr/bin/env python3
"""Ubuntu-side occupancy-map cleanup and geometry correction worker.

The Raspberry Pi remains responsible for live SLAM.  This worker consumes only
completed, checksummed map snapshots.  It removes isolated raster noise,
estimates a conservative global wall skew, joins short wall dropouts, validates
the result, and atomically promotes a corrected navigation map.
"""

from __future__ import annotations

import argparse
import ast
import base64
import hashlib
import json
import math
import os
import time
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


UNKNOWN = np.uint8(205)
FREE = np.uint8(254)
OCCUPIED = np.uint8(0)


@dataclass(frozen=True)
class PostprocessConfig:
    minimum_noise_area_m2: float = 0.0125
    preserve_linear_obstacle_m: float = 0.35
    maximum_fragment_area_m2: float = 0.12
    maximum_unknown_hole_area_m2: float = 0.03
    maximum_wall_gap_m: float = 0.20
    crop_margin_m: float = 0.30
    minimum_hough_line_m: float = 0.45
    maximum_hough_gap_m: float = 0.15
    minimum_angle_lines: int = 3
    maximum_angle_spread_deg: float = 7.0
    maximum_correction_deg: float = 12.0
    minimum_known_retention: float = 0.72
    minimum_free_retention: float = 0.68
    maximum_occupied_growth_ratio: float = 1.45


def _metadata(path: Path) -> dict[str, Any]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        values[key] = value
    origin = ast.literal_eval(values.get("origin", "[0, 0, 0]"))
    result: dict[str, Any] = {
        "resolution": float(values["resolution"]),
        "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
        "negate": int(values.get("negate", "0")),
        "occupied_thresh": float(values.get("occupied_thresh", "0.65")),
        "free_thresh": float(values.get("free_thresh", "0.25")),
    }
    if not math.isfinite(result["resolution"]) or result["resolution"] <= 0.0:
        raise ValueError("map resolution must be positive and finite")
    if not all(math.isfinite(value) for value in result["origin"]):
        raise ValueError("map origin must be finite")
    return result


def load_map(prefix: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    path = Path(prefix)
    if path.suffix.lower() in {".pgm", ".yaml", ".yml"}:
        path = path.with_suffix("")
    image_path = path.with_suffix(".pgm")
    yaml_path = path.with_suffix(".yaml")
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None or image.ndim != 2 or image.size == 0:
        raise ValueError(f"map image is unreadable: {image_path}")
    metadata = _metadata(yaml_path)
    normalized = image if metadata["negate"] == 0 else 255 - image
    occupied_max = round(255.0 * (1.0 - metadata["occupied_thresh"]))
    free_min = round(255.0 * (1.0 - metadata["free_thresh"]))
    trinary = np.full(normalized.shape, UNKNOWN, dtype=np.uint8)
    trinary[normalized <= occupied_max] = OCCUPIED
    trinary[normalized >= free_min] = FREE
    metadata["negate"] = 0
    return trinary, metadata


def _component_stats(mask: np.ndarray) -> tuple[int, np.ndarray, np.ndarray, np.ndarray]:
    return cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)


def remove_noise_sections(
    image: np.ndarray, resolution: float, config: PostprocessConfig
) -> tuple[np.ndarray, dict[str, int]]:
    result = image.copy()
    occupied = result == OCCUPIED
    count, labels, stats, _centroids = _component_stats(occupied)
    minimum_pixels = max(1, math.ceil(config.minimum_noise_area_m2 / resolution**2))
    preserve_pixels = max(1, math.ceil(config.preserve_linear_obstacle_m / resolution))
    remove = np.zeros(result.shape, dtype=bool)
    removed_components = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if area >= minimum_pixels or max(width, height) >= preserve_pixels:
            continue
        remove |= labels == label
        removed_components += 1
    if np.any(remove):
        near_free = cv2.dilate(
            (result == FREE).astype(np.uint8), np.ones((3, 3), np.uint8)
        ).astype(bool)
        result[remove & near_free] = FREE
        result[remove & ~near_free] = UNKNOWN

    known = result != UNKNOWN
    count, labels, stats, _centroids = _component_stats(known)
    fragment_limit = max(
        1, math.ceil(config.maximum_fragment_area_m2 / resolution**2)
    )
    largest_label = (
        1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])) if count > 1 else 0
    )
    removed_fragments = 0
    removed_fragment_pixels = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if label == largest_label or area > fragment_limit:
            continue
        component = labels == label
        removed_fragment_pixels += int(np.count_nonzero(component))
        result[component] = UNKNOWN
        removed_fragments += 1
    return result, {
        "removed_noise_components": removed_components,
        "removed_noise_pixels": int(np.count_nonzero(remove)),
        "removed_known_fragments": removed_fragments,
        "removed_fragment_pixels": removed_fragment_pixels,
    }


def _weighted_median(values: list[float], weights: list[float]) -> float:
    ordered = sorted(zip(values, weights), key=lambda item: item[0])
    halfway = sum(weights) / 2.0
    accumulated = 0.0
    for value, weight in ordered:
        accumulated += weight
        if accumulated >= halfway:
            return value
    return ordered[-1][0]


def estimate_wall_correction(
    image: np.ndarray,
    resolution: float,
    config: PostprocessConfig,
    *,
    clamp_deg: float | None = None,
) -> tuple[float, dict[str, Any]]:
    """Estimate the rotation that squares detected walls to the axes.

    ``clamp_deg`` defaults to ``config.maximum_correction_deg`` (the
    navigation-safe default: a large correction from noisy Hough lines
    could misalign the pose transform used for real navigation). Pass a
    larger value (see ``cutout_room_rectangle``) for a visualization-only
    caller that wants the full correction even past that clamp.
    """

    if clamp_deg is None:
        clamp_deg = config.maximum_correction_deg
    occupied = ((image == OCCUPIED) * 255).astype(np.uint8)
    minimum_length = max(6, round(config.minimum_hough_line_m / resolution))
    maximum_gap = max(1, round(config.maximum_hough_gap_m / resolution))
    lines = cv2.HoughLinesP(
        occupied,
        1,
        np.pi / 360.0,
        threshold=max(12, minimum_length // 2),
        minLineLength=minimum_length,
        maxLineGap=maximum_gap,
    )
    deviations: list[float] = []
    lengths: list[float] = []
    if lines is not None:
        for raw in np.asarray(lines).reshape(-1, 4):
            x1, y1, x2, y2 = map(float, raw)
            length = math.hypot(x2 - x1, y2 - y1)
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
            deviation = (angle + 45.0) % 90.0 - 45.0
            deviations.append(deviation)
            lengths.append(length)
    diagnostics: dict[str, Any] = {
        "hough_line_count": len(deviations),
        "angle_confident": False,
        "angle_spread_deg": None,
    }
    if len(deviations) < config.minimum_angle_lines:
        return 0.0, diagnostics
    center = _weighted_median(deviations, lengths)
    spread = _weighted_median(
        [abs(value - center) for value in deviations], lengths
    )
    diagnostics["angle_spread_deg"] = round(spread, 3)
    if spread > config.maximum_angle_spread_deg:
        return 0.0, diagnostics
    correction = max(-clamp_deg, min(clamp_deg, center))
    if abs(correction) < 0.15:
        correction = 0.0
    diagnostics["angle_confident"] = True
    return correction, diagnostics


def _rotate_map_with_matrix(
    image: np.ndarray,
    metadata: dict[str, Any],
    correction_deg: float,
) -> tuple[np.ndarray, dict[str, Any], np.ndarray]:
    if correction_deg == 0.0:
        return image.copy(), dict(metadata), np.array(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64
        )
    height, width = image.shape
    center = (width / 2.0, height / 2.0)
    matrix = cv2.getRotationMatrix2D(center, correction_deg, 1.0)
    cosine = abs(matrix[0, 0])
    sine = abs(matrix[0, 1])
    output_width = max(1, math.ceil(height * sine + width * cosine))
    output_height = max(1, math.ceil(height * cosine + width * sine))
    matrix[0, 2] += output_width / 2.0 - center[0]
    matrix[1, 2] += output_height / 2.0 - center[1]
    rotated = cv2.warpAffine(
        image,
        matrix,
        (output_width, output_height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=int(UNKNOWN),
    )

    resolution = float(metadata["resolution"])
    origin_x, origin_y, yaw = map(float, metadata["origin"])
    cosine_yaw = math.cos(yaw)
    sine_yaw = math.sin(yaw)
    old_center_x = origin_x + cosine_yaw * width * resolution / 2.0 - sine_yaw * height * resolution / 2.0
    old_center_y = origin_y + sine_yaw * width * resolution / 2.0 + cosine_yaw * height * resolution / 2.0
    local_x = output_width * resolution / 2.0
    local_y = output_height * resolution / 2.0
    updated = dict(metadata)
    updated["origin"] = [
        old_center_x - cosine_yaw * local_x + sine_yaw * local_y,
        old_center_y - sine_yaw * local_x - cosine_yaw * local_y,
        yaw,
    ]
    return rotated, updated, matrix


def rotate_map(
    image: np.ndarray,
    metadata: dict[str, Any],
    correction_deg: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    rotated, updated, _matrix = _rotate_map_with_matrix(
        image, metadata, correction_deg
    )
    return rotated, updated


def _pixel_to_world_matrix(
    shape: tuple[int, int], metadata: dict[str, Any]
) -> np.ndarray:
    height, _width = shape
    resolution = float(metadata["resolution"])
    origin_x, origin_y, yaw = map(float, metadata["origin"])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array(
        [
            [
                cosine * resolution,
                sine * resolution,
                origin_x
                + 0.5 * cosine * resolution
                - sine * (height - 0.5) * resolution,
            ],
            [
                sine * resolution,
                -cosine * resolution,
                origin_y
                + 0.5 * sine * resolution
                + cosine * (height - 0.5) * resolution,
            ],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def coordinate_transform_matrix(
    source_shape: tuple[int, int],
    source_metadata: dict[str, Any],
    destination_shape: tuple[int, int],
    destination_metadata: dict[str, Any],
    pixel_matrix: np.ndarray,
    *,
    crop_left: int,
    crop_top: int,
) -> np.ndarray:
    """Return homogeneous source-map world -> corrected-map world transform."""

    source_pixel_to_world = _pixel_to_world_matrix(source_shape, source_metadata)
    destination_pixel_to_world = _pixel_to_world_matrix(
        destination_shape, destination_metadata
    )
    pixel_affine = np.vstack([pixel_matrix, [0.0, 0.0, 1.0]])
    crop = np.array(
        [[1.0, 0.0, -crop_left], [0.0, 1.0, -crop_top], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    world = (
        destination_pixel_to_world
        @ crop
        @ pixel_affine
        @ np.linalg.inv(source_pixel_to_world)
    )
    if not np.all(np.isfinite(world)) or abs(float(np.linalg.det(world[:2, :2]))) < 0.5:
        raise ValueError("map correction produced an invalid coordinate transform")
    return world


def transform_pose(
    pose: dict[str, Any], transform: list[list[float]] | np.ndarray
) -> dict[str, float]:
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("pose transform must be a finite 3x3 matrix")
    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose["yaw"])
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("source robot pose must be finite")
    transformed = matrix @ np.array([x, y, 1.0], dtype=np.float64)
    heading = matrix[:2, :2] @ np.array(
        [math.cos(yaw), math.sin(yaw)], dtype=np.float64
    )
    corrected_yaw = math.atan2(float(heading[1]), float(heading[0]))
    return {
        "x": round(float(transformed[0]), 6),
        "y": round(float(transformed[1]), 6),
        "yaw": round(corrected_yaw, 6),
    }


def connect_wall_gaps(
    image: np.ndarray, resolution: float, config: PostprocessConfig
) -> tuple[np.ndarray, int]:
    result = image.copy()
    occupied = (result == OCCUPIED).astype(np.uint8)
    gap_pixels = max(1, math.floor(config.maximum_wall_gap_m / resolution))
    kernel_length = gap_pixels + 1
    if kernel_length % 2 == 0:
        kernel_length += 1
    horizontal = cv2.morphologyEx(
        occupied,
        cv2.MORPH_CLOSE,
        np.ones((1, kernel_length), dtype=np.uint8),
    )
    vertical = cv2.morphologyEx(
        occupied,
        cv2.MORPH_CLOSE,
        np.ones((kernel_length, 1), dtype=np.uint8),
    )
    additions = ((horizontal | vertical) != 0) & (occupied == 0)
    result[additions] = OCCUPIED
    return result, int(np.count_nonzero(additions))


def fill_small_unknown_holes(
    image: np.ndarray, resolution: float, config: PostprocessConfig
) -> tuple[np.ndarray, int]:
    result = image.copy()
    unknown = result == UNKNOWN
    count, labels, stats, _centroids = _component_stats(unknown)
    maximum_pixels = max(
        1, math.ceil(config.maximum_unknown_hole_area_m2 / resolution**2)
    )
    filled = 0
    height, width = result.shape
    for label in range(1, count):
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        component_width = int(stats[label, cv2.CC_STAT_WIDTH])
        component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        touches_border = (
            left == 0
            or top == 0
            or left + component_width == width
            or top + component_height == height
        )
        if touches_border or area > maximum_pixels:
            continue
        result[labels == label] = FREE
        filled += area
    return result, filled


def crop_to_known(
    image: np.ndarray, metadata: dict[str, Any], margin_m: float
) -> tuple[np.ndarray, dict[str, Any], dict[str, int]]:
    rows, columns = np.nonzero(image != UNKNOWN)
    if len(rows) == 0:
        raise ValueError("post-processed map contains no known cells")
    resolution = float(metadata["resolution"])
    margin = max(0, math.ceil(margin_m / resolution))
    top = max(0, int(rows.min()) - margin)
    bottom = min(image.shape[0], int(rows.max()) + margin + 1)
    left = max(0, int(columns.min()) - margin)
    right = min(image.shape[1], int(columns.max()) + margin + 1)
    cropped = image[top:bottom, left:right].copy()

    origin_x, origin_y, yaw = map(float, metadata["origin"])
    local_x = left * resolution
    local_y = (image.shape[0] - bottom) * resolution
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    updated = dict(metadata)
    updated["origin"] = [
        origin_x + cosine * local_x - sine * local_y,
        origin_y + sine * local_x + cosine * local_y,
        yaw,
    ]
    return cropped, updated, {
        "crop_top": top,
        "crop_bottom": int(image.shape[0] - bottom),
        "crop_left": left,
        "crop_right": int(image.shape[1] - right),
    }


def cutout_room_rectangle(
    image: np.ndarray,
    metadata: dict[str, Any],
    margin_m: float = 0.10,
    config: PostprocessConfig | None = None,
) -> tuple[np.ndarray, dict[str, Any], dict[str, Any]]:
    """Rotate and crop a corrected map down to just its known rectangle.

    ``estimate_wall_correction``/``process_map`` deliberately clamp their
    rotation to ``maximum_correction_deg`` (12 degrees) since a large
    correction from noisy Hough lines could misalign the safety-critical
    navigation pose transform. That leaves a room whose true skew exceeds
    the clamp still shown as a rotated diamond padded by unknown-space
    corners. This is a separate, visualization-only cutout: it reuses the
    same wall-detecting Hough-line estimate but with the clamp raised to 45
    degrees (the full range the deviation-from-nearest-right-angle math can
    express), rotates by that full correction, and crops tightly to the
    result, leaving just the room's rectangle with a small margin. A
    min-area-rect fit around the *whole* known-pixel blob was tried first
    and rejected -- exploration frontiers and doorway alcoves make that
    outline irregular enough to throw off its angle, where the wall
    segments themselves stay a reliable signal. Not used for the
    navigation pose bundle.
    """

    config = config or PostprocessConfig()
    resolution = float(metadata["resolution"])
    correction_deg, angle_report = estimate_wall_correction(
        image, resolution, config, clamp_deg=45.0
    )
    rotated, rotated_metadata, _matrix = _rotate_map_with_matrix(
        image, metadata, correction_deg
    )
    cropped, final_metadata, crop_report = crop_to_known(
        rotated, rotated_metadata, margin_m
    )
    return (
        cropped,
        final_metadata,
        {
            "cutout_angle_deg": round(correction_deg, 4),
            **angle_report,
            **crop_report,
        },
    )


def _skeletonize(mask: np.ndarray) -> np.ndarray:
    """Thin a binary mask (0/255) down to its 1-pixel-wide centerline."""

    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    skeleton = np.zeros_like(mask)
    working = mask.copy()
    while cv2.countNonZero(working) > 0:
        eroded = cv2.erode(working, element)
        opened = cv2.dilate(eroded, element)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(working, opened))
        working = eroded
    return skeleton


def _largest_component_mask(
    occupied_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, int] | None:
    count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
        occupied_mask, connectivity=8
    )
    if count <= 1:
        return None
    label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels, (labels == label).astype(np.uint8) * 255, label


def _segment_max_deviation(
    points: np.ndarray, start_index: int, end_index: int
) -> float:
    """Max perpendicular distance of a contour stretch from its own chord."""

    n = len(points)
    indices = [start_index]
    i = start_index
    while i != end_index:
        i = (i + 1) % n
        indices.append(i)
    if len(indices) <= 2:
        return 0.0
    start = points[start_index].astype(np.float64)
    end = points[end_index].astype(np.float64)
    chord = end - start
    chord_length = float(np.hypot(*chord))
    if chord_length < 1e-6:
        return float(
            np.max(np.hypot(*(points[indices].astype(np.float64) - start).T))
        )
    normal = np.array([-chord[1], chord[0]]) / chord_length
    offsets = points[indices].astype(np.float64) - start
    return float(np.max(np.abs(offsets @ normal)))


def classify_wall_straight_and_curved(
    image: np.ndarray,
    *,
    max_thickness_px: int = 2,
    straight_tolerance_px: float = 2.0,
    polygon_epsilon_px: float = 10.0,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Redraw the wall as straight lines where straight, curves where curved.

    Visualization only -- the corrected/cropped occupancy data used for
    navigation is untouched by this step. Walls only: the single largest
    connected occupied component. Earlier versions either forced every
    stretch straight (flattened real curves) or forced every stretch
    smoothly curved (blurred real corners); this instead simplifies the
    wall's own contour with ``cv2.approxPolyDP`` to find candidate corners,
    then for each stretch between two consecutive corners checks how far
    the original traced boundary bows away from the straight chord between
    them (``straight_tolerance_px``) -- a stretch within tolerance is
    redrawn as one straight line, a stretch that genuinely bows out is
    redrawn as its own original points (a real curve, left curved).
    ``polygon_epsilon_px`` must stay noticeably coarser than
    ``straight_tolerance_px``: approxPolyDP adapts vertex density to
    curvature, so at a fine epsilon it already places extra vertices along
    a real curve until each individual short chord has tiny deviation --
    every stretch then looks "straight" by construction, misclassifying an
    actual circle as 16 flat segments (caught 2026-08-19). A coarse
    epsilon forces long chords across curved stretches, so their real
    deviation from the arc stays large enough to classify correctly. Drawn
    at ``min(max_thickness_px, the wall's own measured thickness)``. Every
    other occupied pixel (desk and chair marks) is left exactly as it was.
    """

    occupied_mask = ((image == OCCUPIED) * 255).astype(np.uint8)
    found = _largest_component_mask(occupied_mask)
    if found is None:
        return image.copy(), {"straight_segments": 0, "curved_segments": 0}
    _labels, wall_mask, _label = found

    distances = cv2.distanceTransform(wall_mask, cv2.DIST_L2, 3)
    measured_thickness = max(1, int(round(2 * float(np.median(distances[wall_mask > 0])))))
    thickness_px = max(1, min(max_thickness_px, measured_thickness))

    contours, _hierarchy = cv2.findContours(
        wall_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    line_layer = np.zeros_like(occupied_mask)
    straight_segments = 0
    curved_segments = 0
    for contour in contours:
        points = contour.reshape(-1, 2)
        if len(points) < 3:
            continue
        simplified = cv2.approxPolyDP(contour, polygon_epsilon_px, True).reshape(-1, 2)
        if len(simplified) < 2:
            continue
        # approxPolyDP's chosen vertices are a subset of `points` in their
        # original cyclic order, but a jagged real contour can revisit the
        # same pixel coordinate more than once (a tight notch touching
        # itself). Matching purely by coordinate (e.g. np.where's first
        # hit) can then jump backwards to an earlier duplicate and corrupt
        # the whole cyclic walk below, silently dropping stretches of wall
        # (found on the real classroom map, 2026-08-19: only two of its
        # eight sides were ever redrawn). Searching forward from the last
        # match instead respects the contour's actual order.
        n_points = len(points)
        indices = []
        search_start = 0
        for vertex in simplified:
            found_index = None
            for offset in range(n_points):
                candidate = (search_start + offset) % n_points
                if points[candidate][0] == vertex[0] and points[candidate][1] == vertex[1]:
                    found_index = candidate
                    break
            if found_index is None:
                continue
            indices.append(found_index)
            search_start = (found_index + 1) % n_points
        if len(indices) < 2:
            continue
        for i in range(len(indices)):
            start_index = indices[i]
            end_index = indices[(i + 1) % len(indices)]
            deviation = _segment_max_deviation(points, start_index, end_index)
            if deviation <= straight_tolerance_px:
                cv2.line(
                    line_layer,
                    tuple(points[start_index]),
                    tuple(points[end_index]),
                    255,
                    thickness_px,
                )
                straight_segments += 1
            else:
                n = len(points)
                stretch_indices = [start_index]
                j = start_index
                while j != end_index:
                    j = (j + 1) % n
                    stretch_indices.append(j)
                stretch = points[stretch_indices].reshape(-1, 1, 2)
                cv2.polylines(line_layer, [stretch], False, 255, thickness_px)
                curved_segments += 1

    # Each stretch above is drawn independently, and integer-rounded pixel
    # endpoints between adjacent stretches don't always land on the exact
    # same pixel: overlapping strokes at a joint balloon its local
    # thickness well past thickness_px (measured 7.4 px against a target
    # of 2 on the real classroom map, 2026-08-19), while a joint that
    # instead falls a pixel short fragments the loop into dozens of
    # disconnected pieces (53 components measured, should be ~1). Closing
    # small gaps, thinning back to a true centerline, then redilating to
    # the target thickness normalizes every joint the same way regardless
    # of which stretches happened to meet there.
    if np.any(line_layer):
        closed = cv2.morphologyEx(
            line_layer, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        )
        skeleton = _skeletonize(closed)
        thickness_element = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (thickness_px, thickness_px)
        )
        line_layer = cv2.dilate(skeleton, thickness_element)

    styled = image.copy()
    styled[wall_mask > 0] = FREE
    styled[line_layer > 0] = OCCUPIED
    return styled, {
        "straight_segments": straight_segments,
        "curved_segments": curved_segments,
        "thickness_px": thickness_px,
    }


def rectilinearize_wall(
    image: np.ndarray,
    *,
    max_thickness_px: int = 2,
    polygon_epsilon_px: float = 10.0,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Redraw the wall as a single right-angled (no curves) outline.

    Visualization only -- the corrected/cropped occupancy data used for
    navigation is untouched by this step. Walls only: the single largest
    connected occupied component. ``classify_wall_straight_and_curved``
    tried to also preserve genuine curves, drawing each stretch as its own
    line or polyline -- but independently-drawn stretches meet at joints
    whose rounded pixel endpoints don't quite land together, ballooning
    some joints past the target thickness and fragmenting others into
    dozens of disconnected pieces even after a closing/skeletonize cleanup
    pass. A classroom's walls are right angles, not curves, so this skips
    curve detection entirely: the contour is simplified to corner
    candidates, and each real corner is connected to the next by inserting
    one right-angle bend between them (two axis-aligned segments instead
    of one diagonal one) -- the corners themselves are never moved, so
    there is no risk of a moved corner shortcutting across one of a
    non-convex room's real notches. The whole result is drawn in one
    single closed
    ``cv2.polylines`` call -- one continuous stroke has no internal joints
    to mismatch in the first place. Every other occupied pixel (desk and
    chair marks) is left exactly as it was.
    """

    occupied_mask = ((image == OCCUPIED) * 255).astype(np.uint8)
    found = _largest_component_mask(occupied_mask)
    if found is None:
        return image.copy(), {"corners": 0, "thickness_px": 0}
    _labels, wall_mask, _label = found

    distances = cv2.distanceTransform(wall_mask, cv2.DIST_L2, 3)
    measured_thickness = max(1, int(round(2 * float(np.median(distances[wall_mask > 0])))))
    thickness_px = max(1, min(max_thickness_px, measured_thickness))

    contours, _hierarchy = cv2.findContours(
        wall_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    line_layer = np.zeros_like(occupied_mask)
    corners = 0
    for contour in contours:
        simplified = cv2.approxPolyDP(contour, polygon_epsilon_px, True).reshape(-1, 2)
        if len(simplified) < 2:
            continue
        # An earlier version moved each corner to snap the edge into it,
        # chaining off the (already-moved) previous corner. For a simple
        # convex room that mostly works, but this room has two real
        # notches, and the chained snap plus a separate special case for
        # the closing edge could shortcut across one -- drawing a
        # spurious internal line nowhere near the actual wall (found on
        # the real classroom map, 2026-08-19). Keep every real corner
        # exactly where it was measured instead, and connect each
        # consecutive pair with one inserted right-angle bend -- two
        # axis-aligned segments instead of one diagonal one. No corner
        # ever moves, and the same rule applies uniformly to the closing
        # edge (index n-1 back to 0), so there is nothing to special-case.
        n = len(simplified)
        rectilinear = []
        for i in range(n):
            current = simplified[i].astype(np.float64)
            following = simplified[(i + 1) % n].astype(np.float64)
            rectilinear.append(current)
            delta_x = following[0] - current[0]
            delta_y = following[1] - current[1]
            bend = (
                np.array([following[0], current[1]])
                if abs(delta_x) >= abs(delta_y)
                else np.array([current[0], following[1]])
            )
            if not np.array_equal(bend, current) and not np.array_equal(bend, following):
                rectilinear.append(bend)
        polygon = np.round(np.array(rectilinear)).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(line_layer, [polygon], True, 255, thickness_px)
        corners += n

    styled = image.copy()
    styled[wall_mask > 0] = FREE
    styled[line_layer > 0] = OCCUPIED
    return styled, {"corners": corners, "thickness_px": thickness_px}


def mask_outside_wall_as_unknown(image: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    """Hide free-space padding outside the wall; leave the room untouched.

    Visualization only -- the corrected/cropped occupancy data used for
    navigation is untouched by this step. ``cutout_room_rectangle``'s
    rotate-then-crop canvas is an axis-aligned bounding box, which does not
    exactly match a still-diagonal room's true silhouette -- it leaves a
    padding sliver of "free" pixels technically outside the wall but
    inside that box. Anything genuinely outside the wall's own outer
    contour that is currently free is set to unknown instead; the wall
    itself and everything on or inside it (the room's real interior and
    every obstacle mark) is left exactly as it was.
    """

    occupied_mask = ((image == OCCUPIED) * 255).astype(np.uint8)
    found = _largest_component_mask(occupied_mask)
    if found is None:
        return image.copy(), {"masked_pixels": 0}
    _labels, wall_mask, _label = found
    contours, _hierarchy = cv2.findContours(
        wall_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return image.copy(), {"masked_pixels": 0}
    largest_contour = max(contours, key=cv2.contourArea)
    room_region = np.zeros_like(occupied_mask)
    cv2.drawContours(room_region, [largest_contour], -1, 255, thickness=cv2.FILLED)

    outside_and_free = (room_region == 0) & (image == FREE)
    styled = image.copy()
    styled[outside_and_free] = UNKNOWN
    return styled, {"masked_pixels": int(np.count_nonzero(outside_and_free))}


def _quality(image: np.ndarray, resolution: float) -> dict[str, Any]:
    free = int(np.count_nonzero(image == FREE))
    occupied = int(np.count_nonzero(image == OCCUPIED))
    unknown = int(image.size - free - occupied)
    return {
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "free_cells": free,
        "occupied_cells": occupied,
        "known_cells": free + occupied,
        "unknown_cells": unknown,
        "free_area_m2": round(free * resolution**2, 3),
        "occupied_area_m2": round(occupied * resolution**2, 3),
        "known_area_m2": round((free + occupied) * resolution**2, 3),
    }


def validate_result(
    before: dict[str, Any], after: dict[str, Any], config: PostprocessConfig
) -> None:
    if after["known_cells"] < before["known_cells"] * config.minimum_known_retention:
        raise ValueError("post-processing removed too much known map area")
    if after["free_cells"] < before["free_cells"] * config.minimum_free_retention:
        raise ValueError("post-processing removed too much navigable free area")
    allowed_occupied = max(
        before["occupied_cells"] * config.maximum_occupied_growth_ratio,
        before["occupied_cells"] + 20,
    )
    if after["occupied_cells"] > allowed_occupied:
        raise ValueError("wall connection added too much occupied area")
    if after["width"] <= 1 or after["height"] <= 1:
        raise ValueError("post-processed map dimensions are invalid")


def process_map(
    image: np.ndarray,
    metadata: dict[str, Any],
    config: PostprocessConfig | None = None,
) -> tuple[np.ndarray, dict[str, Any], dict[str, Any]]:
    config = config or PostprocessConfig()
    resolution = float(metadata["resolution"])
    before = _quality(image, resolution)
    cleaned, noise_report = remove_noise_sections(image, resolution, config)
    correction_deg, angle_report = estimate_wall_correction(
        cleaned, resolution, config
    )
    rotated, rotated_metadata, pixel_matrix = _rotate_map_with_matrix(
        cleaned, metadata, correction_deg
    )
    connected, connected_pixels = connect_wall_gaps(rotated, resolution, config)
    filled, filled_pixels = fill_small_unknown_holes(
        connected, resolution, config
    )
    cropped, final_metadata, crop_report = crop_to_known(
        filled, rotated_metadata, config.crop_margin_m
    )
    after = _quality(cropped, resolution)
    validate_result(before, after, config)
    world_transform = coordinate_transform_matrix(
        image.shape,
        metadata,
        cropped.shape,
        final_metadata,
        pixel_matrix,
        crop_left=crop_report["crop_left"],
        crop_top=crop_report["crop_top"],
    )
    report: dict[str, Any] = {
        "schema_version": 1,
        "algorithm": "occupancy-clean-align-connect-v1",
        "configuration": asdict(config),
        "before": before,
        "after": after,
        "wall_angle_correction_deg": round(correction_deg, 4),
        "coordinate_transform": [
            [round(float(value), 12) for value in row]
            for row in world_transform.tolist()
        ],
        "connected_wall_pixels": connected_pixels,
        "filled_unknown_hole_pixels": filled_pixels,
        **noise_report,
        **angle_report,
        **crop_report,
    }
    return cropped, final_metadata, report


def _yaml_bytes(image_name: str, metadata: dict[str, Any]) -> bytes:
    origin = metadata["origin"]
    return (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {float(metadata['resolution']):.12g}\n"
        f"origin: [{float(origin[0]):.12g}, {float(origin[1]):.12g}, "
        f"{float(origin[2]):.12g}]\n"
        "negate: 0\n"
        f"occupied_thresh: {float(metadata['occupied_thresh']):.12g}\n"
        f"free_thresh: {float(metadata['free_thresh']):.12g}\n"
    ).encode("utf-8")


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}-{time.time_ns()}")
    try:
        with temporary.open("wb") as output:
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _encoded_pgm(image: np.ndarray) -> bytes:
    ok, encoded = cv2.imencode(".pgm", image)
    if not ok:
        raise RuntimeError("OpenCV could not encode the corrected map")
    return encoded.tobytes()


def _write_map(prefix: Path, image: np.ndarray, metadata: dict[str, Any]) -> None:
    _atomic_write(prefix.with_suffix(".pgm"), _encoded_pgm(image))
    _atomic_write(
        prefix.with_suffix(".yaml"),
        _yaml_bytes(prefix.with_suffix(".pgm").name, metadata),
    )


def navigation_bundle(
    *,
    job: dict[str, Any],
    image: np.ndarray,
    metadata: dict[str, Any],
    corrected_pose: dict[str, float],
    report: dict[str, Any],
) -> dict[str, Any]:
    image_data = _encoded_pgm(image)
    return {
        "schema_version": 1,
        "job_id": str(job["job_id"]),
        "navigation_safe": True,
        "image_base64": base64.b64encode(zlib.compress(image_data, 6)).decode(
            "ascii"
        ),
        "image_encoding": "zlib+base64",
        "corrected_sha256": hashlib.sha256(image_data).hexdigest(),
        "source_sha256": str(job["source_sha256"]),
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "resolution": float(metadata["resolution"]),
        "origin_x": float(metadata["origin"][0]),
        "origin_y": float(metadata["origin"][1]),
        "origin_yaw": float(metadata["origin"][2]),
        "negate": 0,
        "occupied_thresh": float(metadata["occupied_thresh"]),
        "free_thresh": float(metadata["free_thresh"]),
        "robot_pose": corrected_pose,
        "coordinate_transform": report["coordinate_transform"],
        "wall_angle_correction_deg": report["wall_angle_correction_deg"],
    }


def _preview(before: np.ndarray, after: np.ndarray) -> bytes:
    target_height = max(before.shape[0], after.shape[0])
    panels: list[np.ndarray] = []
    for label, image in (("RAW", before), ("CORRECTED", after)):
        scale = target_height / image.shape[0]
        resized = cv2.resize(
            image,
            (max(1, round(image.shape[1] * scale)), target_height),
            interpolation=cv2.INTER_NEAREST,
        )
        panel = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
        cv2.putText(
            panel,
            label,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        panels.append(panel)
    combined = np.hstack(panels)
    ok, encoded = cv2.imencode(".png", combined)
    if not ok:
        raise RuntimeError("OpenCV could not encode the correction preview")
    return encoded.tobytes()


def process_job(root: str | Path, job_path: str | Path) -> dict[str, Any]:
    root_path = Path(root).resolve()
    job_file = Path(job_path)
    job = json.loads(job_file.read_text(encoding="utf-8"))
    input_prefix = (root_path / str(job["input_prefix"])).resolve()
    output_prefix = (root_path / str(job["output_prefix"])).resolve()
    if root_path not in input_prefix.parents or root_path not in output_prefix.parents:
        raise ValueError("map post-processing job escapes the configured map root")

    raw, metadata = load_map(input_prefix)
    corrected, corrected_metadata, report = process_map(raw, metadata)
    source_bytes = input_prefix.with_suffix(".pgm").read_bytes()
    source_digest = hashlib.sha256(source_bytes).hexdigest()
    if source_digest != str(job.get("source_sha256") or ""):
        raise ValueError("archived map checksum changed before post-processing")
    report.update(
        {
            "job_id": str(job["job_id"]),
            "processed_unix": time.time(),
            "source_sha256": source_digest,
            "mapping": job.get("mapping", {}),
        }
    )
    corrected_pose = None
    source_pose = job.get("robot_pose")
    if isinstance(source_pose, dict):
        corrected_pose = transform_pose(source_pose, report["coordinate_transform"])
        report["source_robot_pose"] = source_pose
        report["corrected_robot_pose"] = corrected_pose
        report["navigation_bundle_created"] = True
    else:
        report["navigation_bundle_created"] = False
        report["navigation_bundle_error"] = (
            "raw map did not include the robot pose captured at save time"
        )

    snapshot_prefix = root_path / "corrected" / str(job["job_id"])
    _write_map(snapshot_prefix, corrected, corrected_metadata)
    if corrected_pose is not None:
        _atomic_write(
            snapshot_prefix.with_name(snapshot_prefix.name + "_pose.json"),
            (json.dumps(corrected_pose, separators=(",", ":")) + "\n").encode(),
        )
    _atomic_write(
        snapshot_prefix.with_name(snapshot_prefix.name + "_postprocess.json"),
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True).encode("utf-8"),
    )

    marker = output_prefix.with_name(f".{output_prefix.name}.processing")
    _atomic_write(marker, str(job["job_id"]).encode("utf-8"))
    try:
        _write_map(output_prefix, corrected, corrected_metadata)
        _atomic_write(
            output_prefix.with_name(output_prefix.name + "_postprocess.json"),
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True).encode(
                "utf-8"
            ),
        )
        _atomic_write(
            output_prefix.with_name(output_prefix.name + "_postprocess.png"),
            _preview(raw, corrected),
        )
    finally:
        try:
            marker.unlink()
        except FileNotFoundError:
            pass
    if corrected_pose is not None:
        bundle = navigation_bundle(
            job=job,
            image=corrected,
            metadata=corrected_metadata,
            corrected_pose=corrected_pose,
            report=report,
        )
        _atomic_write(
            root_path / "postprocess-outbox" / f"{job['job_id']}.json",
            json.dumps(bundle, separators=(",", ":"), sort_keys=True).encode(),
        )
    return report


def process_pending_jobs(root: str | Path) -> int:
    root_path = Path(root)
    inbox = root_path / "postprocess-inbox"
    completed = root_path / "postprocess-completed"
    failed = root_path / "postprocess-failed"
    completed.mkdir(parents=True, exist_ok=True)
    failed.mkdir(parents=True, exist_ok=True)
    processed = 0
    for job_path in sorted(inbox.glob("*.json")):
        try:
            process_job(root_path, job_path)
        except Exception as error:
            failure = {
                "job": job_path.name,
                "failed_unix": time.time(),
                "error": str(error),
            }
            _atomic_write(
                failed / job_path.name,
                json.dumps(failure, ensure_ascii=False, indent=2).encode("utf-8"),
            )
            os.replace(job_path, failed / f"{job_path.stem}.job.json")
            print(f"map post-processing failed for {job_path.name}: {error}", flush=True)
            continue
        os.replace(job_path, completed / job_path.name)
        processed += 1
        print(f"map post-processing complete: {job_path.name}", flush=True)
    return processed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        default=os.getenv("MAP_DIRECTORY", "/opt/robot-control/maps"),
        help="shared server map directory",
    )
    parser.add_argument("--input", help="one map prefix for a manual run")
    parser.add_argument("--output", help="corrected map prefix for a manual run")
    parser.add_argument(
        "--cutout",
        action="store_true",
        help="also rotate/crop the corrected map down to just its known "
        "rectangle (visualization only, see cutout_room_rectangle)",
    )
    parser.add_argument(
        "--mask-outside-wall",
        action="store_true",
        help="also hide free-space padding outside the wall as unknown "
        "(visualization only, see mask_outside_wall_as_unknown)",
    )
    parser.add_argument(
        "--classify-wall",
        action="store_true",
        help="also redraw the wall as straight lines where straight and "
        "curves where curved, at up to 2 px thick (visualization only, "
        "see classify_wall_straight_and_curved)",
    )
    parser.add_argument(
        "--rectilinear-wall",
        action="store_true",
        help="also redraw the wall as a single right-angled outline (no "
        "curves), at up to 2 px thick (visualization only, see "
        "rectilinearize_wall)",
    )
    parser.add_argument(
        "--keep-all-marks",
        action="store_true",
        help="disable process_map's small-noise removal for this run "
        "(minimum_noise_area_m2=0) -- 2026-08-19: the default 0.0125 m^2 "
        "threshold was removing 31 of 66 real obstacle marks on one "
        "classroom map, not just sensor noise",
    )
    parser.add_argument("--once", action="store_true", help="process queued jobs once")
    parser.add_argument("--poll-seconds", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if bool(args.input) != bool(args.output):
        raise SystemExit("--input and --output must be specified together")
    if args.input:
        raw, metadata = load_map(args.input)
        run_config = (
            PostprocessConfig(minimum_noise_area_m2=0.0)
            if args.keep_all_marks
            else None
        )
        corrected, corrected_metadata, report = process_map(raw, metadata, run_config)
        if args.cutout:
            corrected, corrected_metadata, cutout_report = cutout_room_rectangle(
                corrected, corrected_metadata
            )
            report["cutout"] = cutout_report
        if args.mask_outside_wall:
            corrected, mask_report = mask_outside_wall_as_unknown(corrected)
            report["mask_outside_wall"] = mask_report
        if args.classify_wall:
            corrected, classify_report = classify_wall_straight_and_curved(corrected)
            report["classify_wall"] = classify_report
        if args.rectilinear_wall:
            corrected, rectilinear_report = rectilinearize_wall(corrected)
            report["rectilinear_wall"] = rectilinear_report
        output = Path(args.output)
        _write_map(output, corrected, corrected_metadata)
        _atomic_write(
            output.with_name(output.name + "_postprocess.json"),
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True).encode(
                "utf-8"
            ),
        )
        _atomic_write(
            output.with_name(output.name + "_postprocess.png"),
            _preview(raw, corrected),
        )
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return
    while True:
        process_pending_jobs(args.root)
        if args.once:
            return
        time.sleep(max(0.25, args.poll_seconds))


if __name__ == "__main__":
    main()
