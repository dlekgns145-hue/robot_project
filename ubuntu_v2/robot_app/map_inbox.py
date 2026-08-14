"""Safely stage a completed robot map for Ubuntu-side post-processing.

The gateway uses only the Python standard library.  It validates and archives
the completed PGM/YAML pair, then atomically publishes a small JSON job.  The
OpenCV worker never observes a partially transferred map.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import time
import zlib
from pathlib import Path
from typing import Any


MAX_COMPRESSED_MAP_BYTES = 16 * 1024 * 1024
MAX_UNCOMPRESSED_MAP_BYTES = 64 * 1024 * 1024


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


def _pgm_dimensions(image_data: bytes) -> tuple[int, int, int]:
    tokens: list[bytes] = []
    for raw_line in image_data.splitlines():
        line = raw_line.split(b"#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
        if len(tokens) >= 4:
            break
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise ValueError("robot map is not a valid PGM image")
    width, height, maximum = map(int, tokens[1:4])
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise ValueError("robot map has an invalid PGM header")
    if width * height > MAX_UNCOMPRESSED_MAP_BYTES:
        raise ValueError("robot map dimensions exceed the server safety limit")
    return width, height, maximum


def decode_map_payload(payload: dict[str, Any]) -> tuple[bytes, dict[str, Any]]:
    encoded = str(payload.get("image_base64") or "")
    if not encoded:
        raise ValueError("robot map payload has no image")
    if len(encoded) > MAX_COMPRESSED_MAP_BYTES * 2:
        raise ValueError("encoded robot map exceeds the server safety limit")
    try:
        compressed = base64.b64decode(encoded, validate=True)
    except (ValueError, TypeError) as error:
        raise ValueError("robot map payload contains invalid base64") from error
    if len(compressed) > MAX_COMPRESSED_MAP_BYTES:
        raise ValueError("compressed robot map exceeds the server safety limit")
    encoding = str(payload.get("image_encoding") or "")
    try:
        if encoding == "zlib+base64":
            decompressor = zlib.decompressobj()
            image_data = decompressor.decompress(
                compressed, MAX_UNCOMPRESSED_MAP_BYTES + 1
            )
            if decompressor.unconsumed_tail or len(image_data) > MAX_UNCOMPRESSED_MAP_BYTES:
                raise ValueError("expanded robot map exceeds the server safety limit")
            image_data += decompressor.flush(
                MAX_UNCOMPRESSED_MAP_BYTES + 1 - len(image_data)
            )
            if len(image_data) > MAX_UNCOMPRESSED_MAP_BYTES:
                raise ValueError("expanded robot map exceeds the server safety limit")
        elif encoding in {"", "base64"}:
            image_data = compressed
        else:
            raise ValueError(f"unsupported robot map encoding: {encoding}")
    except zlib.error as error:
        raise ValueError("robot map compression stream is corrupt") from error

    width, height, _maximum = _pgm_dimensions(image_data)
    declared_width = int(payload.get("width", width))
    declared_height = int(payload.get("height", height))
    if (declared_width, declared_height) != (width, height):
        raise ValueError("robot map dimensions do not match its PGM header")
    resolution = float(payload["resolution"])
    origin = [
        float(payload.get("origin_x", 0.0)),
        float(payload.get("origin_y", 0.0)),
        float(payload.get("origin_yaw", 0.0)),
    ]
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("robot map resolution is invalid")
    if not all(math.isfinite(value) for value in origin):
        raise ValueError("robot map origin is invalid")
    metadata = {
        "resolution": resolution,
        "origin": origin,
        "negate": int(payload.get("negate", 0)),
        "occupied_thresh": float(payload.get("occupied_thresh", 0.65)),
        "free_thresh": float(payload.get("free_thresh", 0.25)),
        "width": width,
        "height": height,
    }
    if not 0.0 < metadata["free_thresh"] < metadata["occupied_thresh"] < 1.0:
        raise ValueError("robot map occupancy thresholds are invalid")
    return image_data, metadata


def render_map_yaml(image_name: str, metadata: dict[str, Any]) -> bytes:
    origin = metadata["origin"]
    return (
        f"image: {image_name}\n"
        f"mode: trinary\n"
        f"resolution: {float(metadata['resolution']):.12g}\n"
        f"origin: [{float(origin[0]):.12g}, {float(origin[1]):.12g}, "
        f"{float(origin[2]):.12g}]\n"
        f"negate: {int(metadata['negate'])}\n"
        f"occupied_thresh: {float(metadata['occupied_thresh']):.12g}\n"
        f"free_thresh: {float(metadata['free_thresh']):.12g}\n"
    ).encode("utf-8")


class MapInbox:
    """Archive completed robot maps and publish atomic worker jobs."""

    def __init__(self, root: str, map_name: str = "orchard_map") -> None:
        self.root = Path(root)
        self.map_name = map_name
        self.archive = self.root / "raw"
        self.incoming = self.root / "postprocess-inbox"
        self.state_path = self.root / ".map-ingest-state.json"

    def _last_digest(self) -> str:
        try:
            state = json.loads(self.state_path.read_text(encoding="utf-8"))
            return str(state.get("sha256") or "")
        except (OSError, ValueError, TypeError):
            return ""

    def stage(
        self, payload: dict[str, Any], mapping_status: dict[str, Any]
    ) -> str | None:
        image_data, metadata = decode_map_payload(payload)
        digest = hashlib.sha256(image_data).hexdigest()
        if digest == self._last_digest():
            return None

        job_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{digest[:12]}"
        raw_prefix = self.archive / job_id
        raw_image = raw_prefix.with_suffix(".pgm")
        raw_yaml = raw_prefix.with_suffix(".yaml")
        _atomic_write(raw_image, image_data)
        _atomic_write(raw_yaml, render_map_yaml(raw_image.name, metadata))

        job = {
            "schema_version": 1,
            "job_id": job_id,
            "created_unix": time.time(),
            "source_sha256": digest,
            "input_prefix": str(raw_prefix.relative_to(self.root)),
            "output_prefix": self.map_name,
            "mapping": {
                "state": str(mapping_status.get("state") or ""),
                "saved_map": str(mapping_status.get("saved_map") or ""),
                "save_sequence": int(mapping_status.get("save_sequence", 0)),
                "travel_distance": float(mapping_status.get("travel_distance", 0.0)),
            },
        }
        robot_pose = payload.get("robot_pose")
        if isinstance(robot_pose, dict):
            try:
                pose = {
                    "x": float(robot_pose["x"]),
                    "y": float(robot_pose["y"]),
                    "yaw": float(robot_pose["yaw"]),
                }
                if all(math.isfinite(value) for value in pose.values()):
                    job["robot_pose"] = pose
            except (KeyError, TypeError, ValueError):
                pass
        job_path = self.incoming / f"{job_id}.json"
        _atomic_write(
            job_path,
            json.dumps(job, ensure_ascii=False, sort_keys=True).encode("utf-8"),
        )
        _atomic_write(
            self.state_path,
            json.dumps(
                {"sha256": digest, "job_id": job_id, "updated_unix": time.time()},
                sort_keys=True,
            ).encode("utf-8"),
        )
        return job_id
