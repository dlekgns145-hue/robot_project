"""map_payload.py - 데스크톱 GUI용 LiDAR 점유 격자 지도(Occupancy Grid) 페이로드 로더
----------------------------------------------------------------------------------
[매핑 기능 커스텀/뜯어고치기 안내]
이 모듈은 로봇에 저장된 *.pgm 및 *.yaml 지도 파일이나 실시간 탐색 중 생성되는 매핑 스트림을 읽어
네트워크(TCP/WebSocket)를 통해 GUI로 효율적으로 전송하기 위한 zlib+base64 압축 데이터 딕셔너리를 구성합니다.

[핵심 기능]
1. _select_map_name():
   - 최신 실시간 자율 매핑이 진행 중이면 `orchard_map_live`를 우선 선택하고,
     정지 상태이거나 저장본을 원할 경우 정적 저장 지도 `orchard_map`을 선택합니다.
2. load_map_payload():
   - PGM 이미지 바이트 데이터를 읽고 zlib 압축 + base64 인코딩하여 페이로드 크기를 90% 이상 압축
   - 해상도(resolution), 원점(origin_x, origin_y, origin_yaw), 문턱값 메타데이터 포함
"""

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
        "occupancy_source": "lidar_slam_only",
        "navigation_safe": True,
    }
    if live_status:
        payload["map_revision"] = int(live_status.get("map_sequence", 0))
        payload["map_updated_unix"] = float(live_status["updated_unix"])
    return payload


def load_configured_map_payload() -> dict[str, Any]:
    return load_map_payload(
        os.getenv("MAP_DIRECTORY", "/opt/robot-control/maps"),
        os.getenv("MAP_NAME", "orchard_map"),
    )
