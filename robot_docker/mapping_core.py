"""Pure mapping health, progress, and durable-save helpers.

This module intentionally has no ROS dependency.  The autonomous mapper uses
it to decide whether a live occupancy grid is usable, detect a stalled Nav2
goal, and promote a map-saver result without exposing half-written artifacts as
the stable navigation map.
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
        return self.resolution * self.resolution

    @property
    def known_area_m2(self) -> float:
        return self.known_cells * self.cell_area

    @property
    def free_area_m2(self) -> float:
        return self.free_cells * self.cell_area

    @property
    def occupied_area_m2(self) -> float:
        return self.occupied_cells * self.cell_area

    @property
    def known_ratio(self) -> float:
        total = self.width * self.height
        return self.known_cells / total if total else 0.0

    def as_dict(self) -> dict[str, int | float]:
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
    """Summarize a ROS occupancy grid using navigation-map thresholds."""

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
    """Return operator-facing reasons that a map is not safe to publish."""

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
    """Track meaningful reductions in Nav2 distance-to-goal."""

    started_at: float
    last_progress_at: float
    best_distance: float | None = None

    @classmethod
    def started(cls, now: float) -> "GoalProgress":
        return cls(started_at=float(now), last_progress_at=float(now))

    def update(self, distance: float, *, now: float, minimum_delta: float) -> bool:
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
        return timeout > 0.0 and now - self.last_progress_at >= timeout


def _pgm_header(path: Path) -> tuple[bytes, int, int, int, int]:
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
    """Validate map-saver output before it can replace the stable map."""

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
    with path.open("rb") as input_file:
        os.fsync(input_file.fileno())


def _fsync_directory(path: Path) -> None:
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
    """Atomically promote validated staging files to the stable map names.

    The PGM is replaced first and the YAML last.  Consumers use the YAML as the
    map entry point, so they never observe metadata that points at a missing
    image.  Existing stable files remain untouched if staging validation fails.
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
    """Remove only the two private staging artifacts for one save attempt."""

    for suffix in (".pgm", ".yaml"):
        try:
            Path(f"{map_prefix}{suffix}").unlink()
        except FileNotFoundError:
            pass
