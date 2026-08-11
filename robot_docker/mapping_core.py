"""mapping_core.py - 점유 격자(Occupancy Grid) 품질 분석, 주행 진척도 추적, 안전한 지도 파일 저장 헬퍼
-----------------------------------------------------------------------------------------
[매핑 기능 커스텀/뜯어고치기 안내]
이 모듈은 ROS 의존성이 없는 순수 Python 데이터 구조 및 검증 헬퍼로 구성되어 있습니다.
자율 매핑 노드(`autonomous_mapping.py`)는 이 모듈을 사용하여 다음 작업을 수행합니다:

1. MapQuality / analyze_occupancy_grid():
   - ROS OccupancyGrid 메세지(점유율 -1~100) 데이터를 전달받아
     주행 가능 영역(Free), 장애물 영역(Occupied), 미탐색 영역(Unknown) 면적(m²) 및 비율을 계산합니다.
   - 탐색 성공/완료 조건이나 맵 발행 기준 면적을 변경하고 싶다면 free_max, occupied_min 문턱값을 조절하세요.

2. quality_failures():
   - 관측된 지도 영역이 너무 작거나 주행 가능 면적이 부족할 때 지도 저장을 차단하는 검증 함수입니다.
   - 최소 탐색 면적 요구사항을 변경하려면 minimum_known_area_m2 기준을 수정하세요.

3. GoalProgress:
   - Nav2 목표 지점으로 이동하는 동안 잔여 거리가 실제로 줄어들고 있는지 추적하여,
     로봇이 특정 위치에서 멈추거나 교착 상태(Stalled)에 빠졌는지를 판별합니다.

4. validate_saved_map() & promote_saved_map():
   - nav2_map_server가 생성한 임시 지도 파일(*.pgm, *.yaml)의 헤더 및 손상 여부를 검증한 후,
     원자적(Atomic) 파일 교체 방식으로 실제 자율주행 지도로 안전하게 승격시킵니다.
"""

from __future__ import annotations

import ast
import math
import os
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class MapQuality:
    """지도 품질 및 영역 통계 데이터 클래스

    Attributes:
        width: 지도 가로 셀(픽셀) 수
        height: 지도 세로 셀(픽셀) 수
        resolution: 셀 1개의 해상도 (미터/셀, 예: 0.05m = 5cm)
        known_cells: 탐색 완료된 총 셀 수 (주행가능 + 장애물 + 불확실)
        free_cells: 이동 가능한 빈 공간 셀 수
        occupied_cells: 장애물이 위치한 셀 수
        uncertain_cells: 점유 확률이 불확실한 셀 수
        unknown_cells: 아직 레이저가 도달하지 않은 미탐색(-1) 셀 수
    """

    width: int
    height: int
    resolution: float
    known_cells: int
    free_cells: int
    occupied_cells: int
    uncertain_cells: int
    unknown_cells: int

    @property
    def cell_area(self) -> float:
        """셀 1개의 면적 (m²)"""
        return self.resolution * self.resolution

    @property
    def known_area_m2(self) -> float:
        """탐색 완료된 면적 (m²)"""
        return self.known_cells * self.cell_area

    @property
    def free_area_m2(self) -> float:
        """이동 가능한 빈 공간 면적 (m²)"""
        return self.free_cells * self.cell_area

    @property
    def occupied_area_m2(self) -> float:
        """장애물 구역 면적 (m²)"""
        return self.occupied_cells * self.cell_area

    @property
    def known_ratio(self) -> float:
        """전체 지도 영역 대비 탐색 완료 비율 (0.0 ~ 1.0)"""
        total = self.width * self.height
        return self.known_cells / total if total else 0.0

    def as_dict(self) -> dict[str, int | float]:
        """GUI 전달용 dictionary 형태 변환"""
        payload = asdict(self)
        payload.update(
            {
                "known_area_m2": round(self.known_area_m2, 3),
                "free_area_m2": round(self.free_area_m2, 3),
                "occupied_area_m2": round(self.occupied_area_m2, 3),
                "known_ratio": round(self.known_ratio, 4),
            }
        )
        return payload


def analyze_occupancy_grid(
    data: Sequence[int],
    *,
    width: int,
    height: int,
    resolution: float,
    free_max: int = 25,
    occupied_min: int = 65,
) -> MapQuality:
    """ROS OccupancyGrid (1차원 array, -1~100)를 해석하여 MapQuality 객체로 반환.

    - raw_value < 0: 미탐색 영역 (-1)
    - raw_value <= free_max (25 이하): 주행 가능 영역 (Free)
    - raw_value >= occupied_min (65 이상): 장애물 영역 (Occupied)
    - 그 외 (26~64): 경계선/불확실 영역 (Uncertain)
    """

    expected = int(width) * int(height)
    if width <= 0 or height <= 0 or len(data) != expected:
        raise ValueError(f"grid data has {len(data)} cells; expected {expected}")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map resolution must be a positive finite number")
    if not 0 <= free_max < occupied_min <= 100:
        raise ValueError("occupancy thresholds are invalid")

    unknown = free = occupied = uncertain = 0
    for raw_value in data:
        value = int(raw_value)
        if value < 0:
            unknown += 1
        elif value <= free_max:
            free += 1
        elif value >= occupied_min:
            occupied += 1
        else:
            uncertain += 1
    return MapQuality(
        width=int(width),
        height=int(height),
        resolution=float(resolution),
        known_cells=free + occupied + uncertain,
        free_cells=free,
        occupied_cells=occupied,
        uncertain_cells=uncertain,
        unknown_cells=unknown,
    )


def quality_failures(
    quality: MapQuality,
    *,
    minimum_known_area_m2: float,
    minimum_free_area_m2: float,
) -> list[str]:
    """지도 저장 또는 내보내기 시 안전성 판정 (미달 이유 리스트 반환).

    탐색된 면적이나 주행 가능 면적이 설정한 최소 기준(minimum_*)보다 작으면 사유를 반환합니다.
    """

    failures: list[str] = []
    if quality.known_area_m2 < max(0.0, minimum_known_area_m2):
        failures.append(
            "observed map area is too small "
            f"({quality.known_area_m2:.2f} < {minimum_known_area_m2:.2f} m^2)"
        )
    if quality.free_area_m2 < max(0.0, minimum_free_area_m2):
        failures.append(
            "known free area is too small "
            f"({quality.free_area_m2:.2f} < {minimum_free_area_m2:.2f} m^2)"
        )
    return failures


@dataclass
class GoalProgress:
    """Nav2 목표 지점 잔여 거리를 모니터링하여 이동 정체/교착(Stall)을 판단하는 클래스"""

    started_at: float
    last_progress_at: float
    best_distance: float | None = None

    @classmethod
    def started(cls, now: float) -> "GoalProgress":
        return cls(started_at=float(now), last_progress_at=float(now))

    def update(self, distance: float, *, now: float, minimum_delta: float) -> bool:
        """목표 거리가 minimum_delta(미터) 이상 유의미하게 단축되었는지 검사 및 갱신"""
        if not math.isfinite(distance) or distance < 0.0:
            return False
        if self.best_distance is None:
            self.best_distance = float(distance)
            self.last_progress_at = float(now)
            return True
        if distance <= self.best_distance - max(0.0, minimum_delta):
            self.best_distance = float(distance)
            self.last_progress_at = float(now)
            return True
        return False

    def stalled(self, *, now: float, timeout: float) -> bool:
        """timeout 초 동안 목표 거리 진척이 없으면 정체(stalled)로 판단"""
        return timeout > 0.0 and now - self.last_progress_at >= timeout


def _pgm_header(path: Path) -> tuple[bytes, int, int, int, int]:
    """PGM 파일의 매직넘버(P2/P5), 가로, 세로, 최대 픽셀값, 픽셀 데이터 시작 오프셋 읽기"""
    tokens: list[bytes] = []
    with path.open("rb") as image_file:
        while len(tokens) < 4:
            line = image_file.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0].strip()
            if line:
                tokens.extend(line.split())
        payload_offset = image_file.tell()
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise ValueError(f"invalid saved PGM map: {path}")
    width, height, maximum = map(int, tokens[1:4])
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise ValueError(f"invalid saved PGM header: {path}")
    return tokens[0], width, height, maximum, payload_offset


def validate_saved_map(
    map_prefix: str,
    *,
    expected_width: int | None = None,
    expected_height: int | None = None,
) -> tuple[Path, Path]:
    """map-saver가 출력한 임시 *.pgm 및 *.yaml 파일의 무결성을 검증."""

    prefix = Path(map_prefix)
    image_path = Path(f"{prefix}.pgm")
    yaml_path = Path(f"{prefix}.yaml")
    if not image_path.is_file() or image_path.stat().st_size < 16:
        raise ValueError("map saver did not produce a usable PGM image")
    if not yaml_path.is_file() or yaml_path.stat().st_size < 16:
        raise ValueError("map saver did not produce usable YAML metadata")
    magic, width, height, maximum, payload_offset = _pgm_header(image_path)
    if magic == b"P5":
        bytes_per_sample = 1 if maximum < 256 else 2
        expected_payload = width * height * bytes_per_sample
        actual_payload = image_path.stat().st_size - payload_offset
        if actual_payload < expected_payload:
            raise ValueError(
                "saved PGM pixel payload is truncated "
                f"({actual_payload} < {expected_payload} bytes)"
            )
    if expected_width is not None and width != expected_width:
        raise ValueError(
            f"saved map width changed unexpectedly ({width} != {expected_width})"
        )
    if expected_height is not None and height != expected_height:
        raise ValueError(
            f"saved map height changed unexpectedly ({height} != {expected_height})"
        )
    metadata = yaml_path.read_text(encoding="utf-8")
    values: dict[str, str] = {}
    for raw_line in metadata.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        values[key] = value
    if any(not values.get(field) for field in ("image", "resolution", "origin")):
        raise ValueError("saved map YAML is missing required metadata")
    try:
        resolution = float(values["resolution"])
        origin = ast.literal_eval(values["origin"])
        origin_values = tuple(float(value) for value in origin)
    except (SyntaxError, TypeError, ValueError) as error:
        raise ValueError("saved map YAML contains invalid geometry") from error
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("saved map YAML resolution is invalid")
    if len(origin_values) != 3 or not all(map(math.isfinite, origin_values)):
        raise ValueError("saved map YAML origin is invalid")
    return image_path, yaml_path


def _rewrite_image_reference(metadata: str, image_name: str) -> str:
    """YAML 파일 내의 image: 경로를 실제 승격될 파일명으로 수정"""
    lines = metadata.splitlines()
    replaced = False
    for index, line in enumerate(lines):
        if line.lstrip().startswith("image:"):
            indentation = line[: len(line) - len(line.lstrip())]
            lines[index] = f"{indentation}image: {image_name}"
            replaced = True
            break
    if not replaced:
        raise ValueError("saved map YAML has no image field")
    return "\n".join(lines) + "\n"


def _fsync_file(path: Path) -> None:
    """파일 내용을 디스크에 즉시 물리적 동기화"""
    with path.open("rb") as input_file:
        os.fsync(input_file.fileno())


def _fsync_directory(path: Path) -> None:
    """디렉토리 항목 변경을 디스크에 물리적 동기화"""
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def promote_saved_map(
    staging_prefix: str,
    stable_prefix: str,
    *,
    expected_width: int | None = None,
    expected_height: int | None = None,
) -> tuple[Path, Path]:
    """임시 저장된 스테이징 지도 파일(staging_prefix.*)을 검증 후 원자적으로 정식 지도(stable_prefix.*)로 교체.

    주행 중 지도 파일 읽기 충돌을 방지하기 위해 파일 복사 -> 무결성 검증 -> os.replace() 방식을 사용합니다.
    """

    staging_image, staging_yaml = validate_saved_map(
        staging_prefix,
        expected_width=expected_width,
        expected_height=expected_height,
    )
    stable = Path(stable_prefix)
    stable.parent.mkdir(parents=True, exist_ok=True)
    stable_image = Path(f"{stable}.pgm")
    stable_yaml = Path(f"{stable}.yaml")
    temporary_image = stable.parent / f".{stable.name}.pgm.tmp"
    temporary_yaml = stable.parent / f".{stable.name}.yaml.tmp"
    rollback_image = stable.parent / f".{stable.name}.pgm.rollback"
    rollback_yaml = stable.parent / f".{stable.name}.yaml.rollback"
    had_stable_image = stable_image.is_file()
    had_stable_yaml = stable_yaml.is_file()

    metadata = _rewrite_image_reference(
        staging_yaml.read_text(encoding="utf-8"), stable_image.name
    )
    try:
        for rollback in (rollback_image, rollback_yaml):
            try:
                rollback.unlink()
            except FileNotFoundError:
                pass
        if had_stable_image:
            shutil.copyfile(stable_image, rollback_image)
            _fsync_file(rollback_image)
        if had_stable_yaml:
            shutil.copyfile(stable_yaml, rollback_yaml)
            _fsync_file(rollback_yaml)
        shutil.copyfile(staging_image, temporary_image)
        _fsync_file(temporary_image)
        temporary_yaml.write_text(metadata, encoding="utf-8")
        _fsync_file(temporary_yaml)
        os.replace(temporary_image, stable_image)
        os.replace(temporary_yaml, stable_yaml)
        _fsync_directory(stable.parent)
    except Exception:
        if rollback_image.is_file():
            os.replace(rollback_image, stable_image)
        elif not had_stable_image:
            try:
                stable_image.unlink()
            except FileNotFoundError:
                pass
        if rollback_yaml.is_file():
            os.replace(rollback_yaml, stable_yaml)
        elif not had_stable_yaml:
            try:
                stable_yaml.unlink()
            except FileNotFoundError:
                pass
        _fsync_directory(stable.parent)
        raise
    finally:
        for temporary in (
            temporary_image,
            temporary_yaml,
            rollback_image,
            rollback_yaml,
        ):
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
    cleanup_saved_map(staging_prefix)
    return stable_image, stable_yaml


def cleanup_saved_map(map_prefix: str) -> None:
    """임시 저장 스테이징 파일(*.pgm, *.yaml) 삭제 정리"""

    for suffix in (".pgm", ".yaml"):
        try:
            Path(f"{map_prefix}{suffix}").unlink()
        except FileNotFoundError:
            pass

