"""Load server-generated occupancy and camera layers for the desktop GUI."""

from __future__ import annotations

import ast
import base64
import json
import os
import zlib
from pathlib import Path
from typing import Any


def _metadata(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        values[key] = value
    return values


def _pgm_dimensions(image_data: bytes) -> tuple[int, int]:
    tokens: list[bytes] = []
    for raw_line in image_data.splitlines():
        line = raw_line.split(b"#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
        if len(tokens) >= 4:
            break
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise ValueError("invalid PGM map")
    return int(tokens[1]), int(tokens[2])


def load_map_payload(directory: str, map_name: str = "orchard_map") -> dict[str, Any]:
    root = Path(directory)
    image_path = root / f"{map_name}.pgm"
    yaml_path = root / f"{map_name}.yaml"
    image_data = image_path.read_bytes()
    metadata = _metadata(yaml_path)
    width, height = _pgm_dimensions(image_data)
    origin = ast.literal_eval(metadata.get("origin", "[0, 0, 0]"))
    payload: dict[str, Any] = {
        "image_base64": base64.b64encode(zlib.compress(image_data, 6)).decode(
            "ascii"
        ),
        "image_encoding": "zlib+base64",
        "width": width,
        "height": height,
        "resolution": float(metadata["resolution"]),
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
        "origin_yaw": float(origin[2]),
        "negate": int(metadata.get("negate", "0")),
        "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
        "free_thresh": float(metadata.get("free_thresh", "0.25")),
    }

    map_mtime = image_path.stat().st_mtime
    binary_layers = {
        "texture_base64": root / f"{map_name}_texture.png",
        "obstacle_texture_base64": root / f"{map_name}_obstacles.png",
    }
    for key, path in binary_layers.items():
        if path.is_file() and path.stat().st_mtime >= map_mtime:
            payload[key] = base64.b64encode(path.read_bytes()).decode("ascii")
    if "texture_base64" in payload:
        payload["texture_format"] = "png"

    materials_path = root / f"{map_name}_materials.json"
    if materials_path.is_file() and materials_path.stat().st_mtime >= map_mtime:
        materials = json.loads(materials_path.read_text(encoding="utf-8"))
        if isinstance(materials, dict):
            payload["obstacle_materials"] = materials
    return payload


def load_configured_map_payload() -> dict[str, Any]:
    return load_map_payload(
        os.getenv("MAP_DIRECTORY", "/opt/robot-control/maps"),
        os.getenv("MAP_NAME", "orchard_map"),
    )
