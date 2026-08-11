#!/usr/bin/env python3
"""Analyze a phone scene capture and create a visual coverage contact sheet."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median

import cv2
import numpy as np


def percentile(values: list[float], q: float) -> float:
    return round(float(np.percentile(np.asarray(values, dtype=np.float32), q)), 2)


def feature_overlap(first: np.ndarray, second: np.ndarray) -> dict[str, float | int | bool]:
    orb = cv2.ORB_create(nfeatures=1200)
    gray_a = cv2.cvtColor(first, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(second, cv2.COLOR_BGR2GRAY)
    key_a, desc_a = orb.detectAndCompute(gray_a, None)
    key_b, desc_b = orb.detectAndCompute(gray_b, None)
    if desc_a is None or desc_b is None or len(key_a) < 8 or len(key_b) < 8:
        return {"good_matches": 0, "inliers": 0, "inlier_ratio": 0.0, "usable": False}

    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(desc_a, desc_b, k=2)
    good = [left for left, right in pairs if left.distance < 0.72 * right.distance]
    inliers = 0
    if len(good) >= 8:
        points_a = np.float32([key_a[item.queryIdx].pt for item in good]).reshape(-1, 1, 2)
        points_b = np.float32([key_b[item.trainIdx].pt for item in good]).reshape(-1, 1, 2)
        _, mask = cv2.findHomography(points_a, points_b, cv2.RANSAC, 4.0)
        if mask is not None:
            inliers = int(mask.sum())
    ratio = round(inliers / max(len(good), 1), 3)
    return {
        "good_matches": len(good),
        "inliers": inliers,
        "inlier_ratio": ratio,
        "usable": len(good) >= 20 and inliers >= 12 and ratio >= 0.25,
    }


def make_contact_sheet(images: list[np.ndarray], labels: list[str], output: Path) -> None:
    columns = 4
    cell_width, cell_height = 400, 300
    rows = math.ceil(len(images) / columns)
    sheet = np.full((rows * cell_height, columns * cell_width, 3), 24, dtype=np.uint8)
    for index, (image, label) in enumerate(zip(images, labels)):
        row, column = divmod(index, columns)
        scale = min(cell_width / image.shape[1], (cell_height - 30) / image.shape[0])
        width, height = int(image.shape[1] * scale), int(image.shape[0] * scale)
        resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        x = column * cell_width + (cell_width - width) // 2
        y = row * cell_height + 30 + (cell_height - 30 - height) // 2
        sheet[y : y + height, x : x + width] = resized
        cv2.putText(
            sheet,
            label,
            (column * cell_width + 10, row * cell_height + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), sheet, [cv2.IMWRITE_JPEG_QUALITY, 90]):
        raise RuntimeError(f"failed to write {output}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--samples", type=int, default=16)
    args = parser.parse_args()

    session_dir = args.session_dir.resolve()
    output_dir = (args.output_dir or session_dir).resolve()
    manifest = json.loads((session_dir / "manifest.json").read_text(encoding="utf-8"))
    frames = manifest.get("frames", [])
    if not frames:
        raise SystemExit("capture manifest contains no frames")

    images = []
    for frame in frames:
        image = cv2.imread(str(session_dir / frame["file"]))
        if image is None:
            raise SystemExit(f"cannot read {frame['file']}")
        images.append(image)

    sample_count = min(max(args.samples, 1), len(images))
    sample_indexes = sorted(set(np.linspace(0, len(images) - 1, sample_count, dtype=int).tolist()))
    labels = [
        f"#{frames[i]['index']:03d}  t={frames[i]['elapsed_seconds']:.1f}s"
        for i in sample_indexes
    ]
    make_contact_sheet(
        [images[i] for i in sample_indexes],
        labels,
        output_dir / "contact_sheet.jpg",
    )

    overlaps = [feature_overlap(images[i - 1], images[i]) for i in range(1, len(images))]
    sharpness = [float(frame["sharpness"]) for frame in frames]
    brightness = [float(frame["brightness"]) for frame in frames]
    changes = [float(frame["change_score"]) for frame in frames[1:]]
    usable = sum(bool(item["usable"]) for item in overlaps)
    first_last = feature_overlap(images[0], images[-1])

    active_indexes = [
        index
        for index, frame in enumerate(frames)
        if index > 0 and float(frame["change_score"]) >= 10.0
    ]
    if active_indexes:
        active_start = max(active_indexes[0] - 1, 0)
        active_end = min(active_indexes[-1] + 1, len(images) - 1)
    else:
        active_start, active_end = 0, len(images) - 1
    active_sample_count = min(max(args.samples, 1), active_end - active_start + 1)
    active_sample_indexes = sorted(
        set(np.linspace(active_start, active_end, active_sample_count, dtype=int).tolist())
    )
    make_contact_sheet(
        [images[i] for i in active_sample_indexes],
        [
            f"#{frames[i]['index']:03d}  t={frames[i]['elapsed_seconds']:.1f}s"
            for i in active_sample_indexes
        ],
        output_dir / "active_contact_sheet.jpg",
    )

    report = {
        "schema": "robot-phone-scene-analysis-v1",
        "session": session_dir.name,
        "frame_count": len(frames),
        "duration_seconds": manifest.get("elapsed_seconds"),
        "source_size": manifest.get("source_size"),
        "quality": {
            "sharpness_min": round(min(sharpness), 2),
            "sharpness_median": round(median(sharpness), 2),
            "sharpness_p90": percentile(sharpness, 90),
            "brightness_min": round(min(brightness), 2),
            "brightness_median": round(median(brightness), 2),
            "brightness_max": round(max(brightness), 2),
            "change_score_median": round(median(changes), 2),
            "change_score_p90": percentile(changes, 90),
        },
        "visual_overlap": {
            "adjacent_pairs": len(overlaps),
            "usable_pairs": usable,
            "usable_ratio": round(usable / max(len(overlaps), 1), 3),
            "median_good_matches": round(median(int(item["good_matches"]) for item in overlaps), 1),
            "median_inliers": round(median(int(item["inliers"]) for item in overlaps), 1),
            "first_to_last": first_last,
        },
        "active_capture_window": {
            "change_score_threshold": 10.0,
            "first_frame_index": int(frames[active_start]["index"]),
            "last_frame_index": int(frames[active_end]["index"]),
            "frame_count": active_end - active_start + 1,
            "start_seconds": frames[active_start]["elapsed_seconds"],
            "end_seconds": frames[active_end]["elapsed_seconds"],
        },
        "sample_frame_indexes": [int(frames[i]["index"]) for i in sample_indexes],
        "spatial_registration": manifest.get("spatial_registration"),
        "outputs": {
            "contact_sheet": "contact_sheet.jpg",
            "active_contact_sheet": "active_contact_sheet.jpg",
        },
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "coverage_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
