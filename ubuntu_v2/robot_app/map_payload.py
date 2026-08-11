"""Load server-generated occupancy and camera layers for the desktop GUI."""

from __future__ import annotations

import ast
import base64
import json
import math
import os
import time
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


def _select_map_name(
    root: Path,
    map_name: str,
    *,
    now_unix: float,
    live_max_age: float,
) -> tuple[str, dict[str, Any]]:
    """Prefer a fresh, explicitly active live map over the saved map."""
    live_status_path = root / f"{map_name}_live_status.json"
    if live_status_path.is_file():
        try:
            status = json.loads(live_status_path.read_text(encoding="utf-8"))
            if not isinstance(status, dict):
                return map_name, {}
            updated = float(status.get("updated_unix", 0.0))
            age = now_unix - updated
            live_name = f"{map_name}_live"
            live_exists = (root / f"{live_name}.pgm").is_file() and (
                root / f"{live_name}.yaml"
            ).is_file()
            if (
                bool(status.get("active", False))
                and math.isfinite(updated)
                and -2.0 <= age <= live_max_age
                and live_exists
            ):
                return live_name, status
        except (json.JSONDecodeError, OSError, TypeError, ValueError):
            pass
    return map_name, {}


def load_map_payload(
    directory: str,
    map_name: str = "orchard_map",
    *,
    now_unix: float | None = None,
    live_max_age: float = 8.0,
) -> dict[str, Any]:
    root = Path(directory)
    selected_name, live_status = _select_map_name(
        root,
        map_name,
        now_unix=time.time() if now_unix is None else float(now_unix),
        live_max_age=max(1.0, float(live_max_age)),
    )
    image_path = root / f"{selected_name}.pgm"
    yaml_path = root / f"{selected_name}.yaml"
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
        "map_source": "live" if selected_name != map_name else "saved",
        "navigation_safe": True,
    }
    if live_status:
        payload["layer_revision"] = int(live_status.get("map_sequence", 0))
        payload["layer_updated_unix"] = float(live_status["updated_unix"])
        payload["visual_layer_navigation_safe"] = False
        payload["occupancy_source"] = str(
            live_status.get("occupancy_source", "lidar_slam_only")
        )

    map_mtime = image_path.stat().st_mtime
    binary_layers = {
        "texture_base64": root / f"{selected_name}_texture.png",
        "obstacle_texture_base64": root / f"{selected_name}_obstacles.png",
    }
    for key, path in binary_layers.items():
        if path.is_file() and path.stat().st_mtime >= map_mtime:
            payload[key] = base64.b64encode(path.read_bytes()).decode("ascii")
    if "texture_base64" in payload:
        payload["texture_format"] = "png"
    if any(key in payload for key in binary_layers):
        payload["visual_layer_navigation_safe"] = False
        payload.setdefault("occupancy_source", "lidar_slam_only")

    materials_path = root / f"{selected_name}_materials.json"
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
