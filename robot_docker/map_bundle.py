"""Validate and atomically install a server-corrected map/pose bundle on Pi."""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import re
import shutil
import time
import zlib
from pathlib import Path
from typing import Any


MAX_COMPRESSED_BYTES = 16 * 1024 * 1024
MAX_IMAGE_BYTES = 64 * 1024 * 1024
JOB_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,99}$")


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


def _pgm_dimensions(image: bytes) -> tuple[int, int]:
    tokens: list[bytes] = []
    for raw_line in image.splitlines():
        line = raw_line.split(b"#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
        if len(tokens) >= 4:
            break
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise ValueError("corrected map is not a valid PGM")
    width, height, maximum = map(int, tokens[1:4])
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise ValueError("corrected map has an invalid PGM header")
    if width * height > MAX_IMAGE_BYTES:
        raise ValueError("corrected map dimensions exceed the Pi safety limit")
    return width, height


def _decode_image(bundle: dict[str, Any]) -> bytes:
    encoded = str(bundle.get("image_base64") or "")
    if not encoded or len(encoded) > MAX_COMPRESSED_BYTES * 2:
        raise ValueError("corrected map image payload is missing or too large")
    try:
        compressed = base64.b64decode(encoded, validate=True)
    except (TypeError, ValueError) as error:
        raise ValueError("corrected map contains invalid base64") from error
    if len(compressed) > MAX_COMPRESSED_BYTES:
        raise ValueError("compressed corrected map exceeds the Pi safety limit")
    if bundle.get("image_encoding") != "zlib+base64":
        raise ValueError("corrected map must use zlib+base64")
    try:
        stream = zlib.decompressobj()
        image = stream.decompress(compressed, MAX_IMAGE_BYTES + 1)
        if stream.unconsumed_tail or len(image) > MAX_IMAGE_BYTES:
            raise ValueError("expanded corrected map exceeds the Pi safety limit")
        image += stream.flush(MAX_IMAGE_BYTES + 1 - len(image))
    except zlib.error as error:
        raise ValueError("corrected map compression stream is corrupt") from error
    if len(image) > MAX_IMAGE_BYTES:
        raise ValueError("expanded corrected map exceeds the Pi safety limit")
    return image


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_navigation_bundle(
    bundle: dict[str, Any],
) -> tuple[bytes, dict[str, Any], dict[str, float], list[list[float]]]:
    if not isinstance(bundle, dict) or not bool(bundle.get("navigation_safe", False)):
        raise ValueError("server did not mark the corrected bundle navigation-safe")
    job_id = str(bundle.get("job_id") or "")
    if JOB_ID.fullmatch(job_id) is None:
        raise ValueError("corrected map job id is invalid")
    image = _decode_image(bundle)
    width, height = _pgm_dimensions(image)
    if (int(bundle.get("width", width)), int(bundle.get("height", height))) != (
        width,
        height,
    ):
        raise ValueError("corrected map dimensions do not match its PGM")
    digest = hashlib.sha256(image).hexdigest()
    if digest != str(bundle.get("corrected_sha256") or ""):
        raise ValueError("corrected map checksum does not match")

    resolution = _finite_float(bundle.get("resolution"), "resolution")
    if resolution <= 0.0:
        raise ValueError("corrected map resolution must be positive")
    origin = [
        _finite_float(bundle.get("origin_x"), "origin_x"),
        _finite_float(bundle.get("origin_y"), "origin_y"),
        _finite_float(bundle.get("origin_yaw", 0.0), "origin_yaw"),
    ]
    free_thresh = _finite_float(bundle.get("free_thresh", 0.25), "free_thresh")
    occupied_thresh = _finite_float(
        bundle.get("occupied_thresh", 0.65), "occupied_thresh"
    )
    if not 0.0 < free_thresh < occupied_thresh < 1.0:
        raise ValueError("corrected map occupancy thresholds are invalid")
    metadata = {
        "job_id": job_id,
        "sha256": digest,
        "source_sha256": str(bundle.get("source_sha256") or ""),
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
        "free_thresh": free_thresh,
        "occupied_thresh": occupied_thresh,
    }

    raw_pose = bundle.get("robot_pose")
    if not isinstance(raw_pose, dict):
        raise ValueError("corrected bundle has no transformed robot pose")
    pose = {
        "x": _finite_float(raw_pose.get("x"), "robot_pose.x"),
        "y": _finite_float(raw_pose.get("y"), "robot_pose.y"),
        "yaw": _finite_float(raw_pose.get("yaw"), "robot_pose.yaw"),
    }
    if abs(pose["x"]) > 100.0 or abs(pose["y"]) > 100.0:
        raise ValueError("corrected robot pose is outside the supported map range")
    pose["yaw"] = math.atan2(math.sin(pose["yaw"]), math.cos(pose["yaw"]))

    raw_transform = bundle.get("coordinate_transform")
    if (
        not isinstance(raw_transform, list)
        or len(raw_transform) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in raw_transform)
    ):
        raise ValueError("corrected bundle has no 3x3 coordinate transform")
    transform = [
        [_finite_float(value, "coordinate_transform") for value in row]
        for row in raw_transform
    ]
    affine_tail = zip(transform[2], (0.0, 0.0, 1.0))
    if any(abs(value - expected) > 1e-6 for value, expected in affine_tail):
        raise ValueError("corrected coordinate transform is not affine")
    return image, metadata, pose, transform


def _map_yaml(metadata: dict[str, Any]) -> bytes:
    origin = metadata["origin"]
    return (
        "image: map.pgm\n"
        "mode: trinary\n"
        f"resolution: {metadata['resolution']:.12g}\n"
        f"origin: [{origin[0]:.12g}, {origin[1]:.12g}, {origin[2]:.12g}]\n"
        "negate: 0\n"
        f"occupied_thresh: {metadata['occupied_thresh']:.12g}\n"
        f"free_thresh: {metadata['free_thresh']:.12g}\n"
    ).encode("utf-8")


def install_navigation_bundle(
    bundle: dict[str, Any], map_directory: str
) -> dict[str, Any]:
    image, metadata, pose, transform = validate_navigation_bundle(bundle)
    root = Path(map_directory)
    corrected_root = root / "corrected"
    target = corrected_root / metadata["job_id"]
    manifest = {
        "schema_version": 1,
        "job_id": metadata["job_id"],
        "corrected_sha256": metadata["sha256"],
        "source_sha256": metadata["source_sha256"],
        "installed_unix": time.time(),
        "coordinate_transform": transform,
        "pose": pose,
    }
    corrected_root.mkdir(parents=True, exist_ok=True)
    if target.exists():
        try:
            existing = json.loads((target / "manifest.json").read_text())
        except (OSError, ValueError) as error:
            raise ValueError("existing corrected-map directory is incomplete") from error
        if existing.get("corrected_sha256") != metadata["sha256"]:
            raise ValueError("corrected-map job id was reused with different content")
        manifest = existing
    else:
        temporary = corrected_root / f".pending-{metadata['job_id']}-{os.getpid()}"
        if temporary.exists():
            shutil.rmtree(temporary)
        temporary.mkdir()
        try:
            _atomic_write(temporary / "map.pgm", image)
            _atomic_write(temporary / "map.yaml", _map_yaml(metadata))
            _atomic_write(
                temporary / "pose.json",
                (json.dumps(pose, separators=(",", ":")) + "\n").encode(),
            )
            _atomic_write(
                temporary / "manifest.json",
                (json.dumps(manifest, separators=(",", ":")) + "\n").encode(),
            )
            os.replace(temporary, target)
        finally:
            if temporary.exists():
                shutil.rmtree(temporary)

    current = root / "navigation-current"
    temporary_link = root / f".navigation-current.tmp-{os.getpid()}"
    try:
        temporary_link.unlink()
    except FileNotFoundError:
        pass
    os.symlink(str(Path("corrected") / metadata["job_id"]), temporary_link)
    os.replace(temporary_link, current)
    _atomic_write(
        root / "navigation_bundle_status.json",
        (json.dumps(manifest, separators=(",", ":")) + "\n").encode(),
    )
    return manifest


def active_navigation_paths(map_directory: str) -> tuple[Path, Path] | None:
    root = Path(map_directory).resolve()
    current = root / "navigation-current"
    map_yaml = current / "map.yaml"
    pose_json = current / "pose.json"
    if not map_yaml.is_file() or not pose_json.is_file():
        return None
    resolved = current.resolve()
    if root not in resolved.parents or resolved.parent != root / "corrected":
        raise ValueError("active navigation map link escapes the corrected-map store")
    return map_yaml, pose_json
