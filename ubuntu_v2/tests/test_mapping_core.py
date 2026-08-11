from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


DOCKER_DIR = Path(__file__).resolve().parents[2] / "robot_docker"
sys.path.insert(0, str(DOCKER_DIR))

from mapping_core import (  # noqa: E402
    GoalProgress,
    analyze_occupancy_grid,
    promote_saved_map,
    quality_failures,
    validate_saved_map,
)


def write_saved_map(prefix: Path, *, width: int = 4, height: int = 2) -> None:
    pixels = bytes(range(width * height))
    Path(f"{prefix}.pgm").write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + pixels
    )
    Path(f"{prefix}.yaml").write_text(
        "\n".join(
            (
                f"image: {prefix.name}.pgm",
                "resolution: 0.05",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            )
        ),
        encoding="utf-8",
    )


class MappingQualityTests(unittest.TestCase):
    def test_quality_uses_metric_area_instead_of_raw_cell_count(self) -> None:
        quality = analyze_occupancy_grid(
            [-1, 0, 10, 50, 65, 100],
            width=3,
            height=2,
            resolution=0.5,
        )

        self.assertEqual(quality.unknown_cells, 1)
        self.assertEqual(quality.free_cells, 2)
        self.assertEqual(quality.uncertain_cells, 1)
        self.assertEqual(quality.occupied_cells, 2)
        self.assertAlmostEqual(quality.known_area_m2, 1.25)
        self.assertAlmostEqual(quality.free_area_m2, 0.50)

    def test_tiny_or_obstacle_only_map_is_rejected(self) -> None:
        quality = analyze_occupancy_grid(
            [100] * 4,
            width=2,
            height=2,
            resolution=0.1,
        )

        failures = quality_failures(
            quality,
            minimum_known_area_m2=1.0,
            minimum_free_area_m2=0.5,
        )

        self.assertEqual(len(failures), 2)
        self.assertIn("observed map area", failures[0])
        self.assertIn("known free area", failures[1])

    def test_invalid_grid_shape_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            analyze_occupancy_grid([0], width=2, height=2, resolution=0.05)


class GoalProgressTests(unittest.TestCase):
    def test_only_meaningful_distance_reduction_resets_watchdog(self) -> None:
        progress = GoalProgress.started(10.0)

        self.assertTrue(progress.update(2.0, now=11.0, minimum_delta=0.1))
        self.assertFalse(progress.update(1.96, now=20.0, minimum_delta=0.1))
        self.assertTrue(progress.update(1.85, now=21.0, minimum_delta=0.1))
        self.assertFalse(progress.stalled(now=30.0, timeout=10.0))
        self.assertTrue(progress.stalled(now=31.0, timeout=10.0))

    def test_invalid_feedback_does_not_count_as_progress(self) -> None:
        progress = GoalProgress.started(5.0)

        self.assertFalse(progress.update(float("nan"), now=6.0, minimum_delta=0.1))
        self.assertFalse(progress.update(-1.0, now=7.0, minimum_delta=0.1))
        self.assertTrue(progress.stalled(now=10.0, timeout=5.0))


class DurableMapSaveTests(unittest.TestCase):
    def test_valid_staging_map_is_promoted_and_reference_is_rewritten(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "orchard_map.pending-1"
            stable = root / "orchard_map"
            write_saved_map(staging)

            image, metadata = promote_saved_map(str(staging), str(stable))

            self.assertEqual(image, Path(f"{stable}.pgm"))
            self.assertEqual(metadata, Path(f"{stable}.yaml"))
            self.assertIn("image: orchard_map.pgm", metadata.read_text())
            self.assertFalse(Path(f"{staging}.pgm").exists())
            self.assertFalse(Path(f"{staging}.yaml").exists())
            validate_saved_map(str(stable), expected_width=4, expected_height=2)

    def test_truncated_staging_map_cannot_overwrite_stable_map(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "orchard_map.pending-1"
            stable = root / "orchard_map"
            write_saved_map(stable)
            old_image = Path(f"{stable}.pgm").read_bytes()
            old_yaml = Path(f"{stable}.yaml").read_text()
            write_saved_map(staging, width=10, height=10)
            Path(f"{staging}.pgm").write_bytes(
                b"P5\n10 10\n255\n" + bytes(range(20))
            )

            with self.assertRaisesRegex(ValueError, "truncated"):
                promote_saved_map(str(staging), str(stable))

            self.assertEqual(Path(f"{stable}.pgm").read_bytes(), old_image)
            self.assertEqual(Path(f"{stable}.yaml").read_text(), old_yaml)

    def test_promotion_failure_rolls_back_both_existing_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "orchard_map.pending-1"
            stable = root / "orchard_map"
            write_saved_map(stable, width=4, height=2)
            old_image = Path(f"{stable}.pgm").read_bytes()
            old_yaml = Path(f"{stable}.yaml").read_text()
            write_saved_map(staging, width=6, height=3)

            with patch("mapping_core._fsync_directory", side_effect=OSError("disk")):
                with self.assertRaisesRegex(OSError, "disk"):
                    promote_saved_map(str(staging), str(stable))

            self.assertEqual(Path(f"{stable}.pgm").read_bytes(), old_image)
            self.assertEqual(Path(f"{stable}.yaml").read_text(), old_yaml)


if __name__ == "__main__":
    unittest.main()
