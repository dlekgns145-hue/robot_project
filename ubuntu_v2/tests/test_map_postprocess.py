from __future__ import annotations

import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

APP_DIR = Path(__file__).resolve().parents[1] / "robot_app"
sys.path.insert(0, str(APP_DIR))

from map_postprocess import (  # noqa: E402
    FREE,
    OCCUPIED,
    UNKNOWN,
    PostprocessConfig,
    connect_wall_gaps,
    estimate_wall_correction,
    process_pending_jobs,
    remove_noise_sections,
    rotate_map,
    transform_pose,
)


class ServerMapPostprocessTests(unittest.TestCase):
    def test_isolated_noise_is_removed_but_linear_wall_is_preserved(self) -> None:
        image = np.full((40, 40), FREE, dtype=np.uint8)
        image[20, 5:30] = OCCUPIED
        image[4, 4] = OCCUPIED
        image[35, 32] = OCCUPIED

        cleaned, report = remove_noise_sections(
            image, 0.05, PostprocessConfig()
        )

        self.assertEqual(int(cleaned[20, 10]), int(OCCUPIED))
        self.assertEqual(int(cleaned[4, 4]), int(FREE))
        self.assertGreaterEqual(report["removed_noise_pixels"], 2)

    def test_short_horizontal_and_vertical_wall_dropouts_are_joined(self) -> None:
        image = np.full((50, 50), UNKNOWN, dtype=np.uint8)
        image[10, 5:20] = OCCUPIED
        image[10, 23:40] = OCCUPIED
        image[5:20, 30] = OCCUPIED
        image[23:40, 30] = OCCUPIED

        connected, additions = connect_wall_gaps(
            image, 0.05, PostprocessConfig(maximum_wall_gap_m=0.20)
        )

        self.assertTrue(np.all(connected[10, 20:23] == OCCUPIED))
        self.assertTrue(np.all(connected[20:23, 30] == OCCUPIED))
        self.assertGreaterEqual(additions, 6)

    def test_dominant_skew_is_detected_conservatively(self) -> None:
        image = np.full((180, 180), FREE, dtype=np.uint8)
        angle = math.radians(7.0)
        for offset in (35, 70, 105, 140):
            x1, x2 = 15, 165
            y1 = offset
            y2 = round(offset + math.tan(angle) * (x2 - x1))
            cv2.line(image, (x1, y1), (x2, y2), int(OCCUPIED), 2)

        correction, report = estimate_wall_correction(
            image, 0.05, PostprocessConfig()
        )

        self.assertTrue(report["angle_confident"])
        self.assertAlmostEqual(correction, 7.0, delta=1.0)
        rotated, _metadata = rotate_map(
            image,
            {
                "resolution": 0.05,
                "origin": [0.0, 0.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
            correction,
        )
        residual, _report = estimate_wall_correction(
            rotated, 0.05, PostprocessConfig()
        )
        self.assertAlmostEqual(residual, 0.0, delta=0.5)

    def test_crop_only_keeps_world_pose_coordinates_unchanged(self) -> None:
        image = np.full((100, 120), UNKNOWN, dtype=np.uint8)
        image[20:80, 25:95] = FREE
        image[20, 25:95] = OCCUPIED
        image[79, 25:95] = OCCUPIED
        image[20:80, 25] = OCCUPIED
        image[20:80, 94] = OCCUPIED
        from map_postprocess import process_map

        _corrected, _metadata, report = process_map(
            image,
            {
                "resolution": 0.05,
                "origin": [-2.0, -3.0, 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            },
        )
        pose = {"x": 0.2, "y": -0.4, "yaw": 0.7}
        transformed = transform_pose(pose, report["coordinate_transform"])

        self.assertAlmostEqual(transformed["x"], pose["x"], places=5)
        self.assertAlmostEqual(transformed["y"], pose["y"], places=5)
        self.assertAlmostEqual(transformed["yaw"], pose["yaw"], places=5)

    def test_job_preserves_raw_map_and_promotes_corrected_pair(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw_dir = root / "raw"
            inbox = root / "postprocess-inbox"
            raw_dir.mkdir()
            inbox.mkdir()
            image = np.full((80, 100), UNKNOWN, dtype=np.uint8)
            image[10:70, 10:90] = FREE
            image[10, 10:45] = OCCUPIED
            image[10, 48:90] = OCCUPIED
            image[69, 10:90] = OCCUPIED
            image[10:70, 10] = OCCUPIED
            image[10:70, 89] = OCCUPIED
            ok, encoded = cv2.imencode(".pgm", image)
            self.assertTrue(ok)
            raw_bytes = encoded.tobytes()
            (raw_dir / "job.pgm").write_bytes(raw_bytes)
            (raw_dir / "job.yaml").write_text(
                "image: job.pgm\nresolution: 0.05\n"
                "origin: [-1.0, -2.0, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
            )
            job = {
                "job_id": "job",
                "input_prefix": "raw/job",
                "output_prefix": "orchard_map",
                "source_sha256": hashlib.sha256(raw_bytes).hexdigest(),
                "mapping": {"state": "completed", "save_sequence": 1},
                "robot_pose": {"x": 0.5, "y": -0.25, "yaw": 0.3},
            }
            job_path = inbox / "job.json"
            job_path.write_text(json.dumps(job))

            self.assertEqual(process_pending_jobs(root), 1)
            report = json.loads(
                (root / "orchard_map_postprocess.json").read_text()
            )

            self.assertTrue((root / "orchard_map.pgm").is_file())
            self.assertTrue((root / "orchard_map.yaml").is_file())
            self.assertTrue((root / "orchard_map_postprocess.png").is_file())
            self.assertTrue((root / "corrected/job.pgm").is_file())
            self.assertTrue((root / "postprocess-completed/job.json").is_file())
            self.assertTrue((root / "postprocess-outbox/job.json").is_file())
            self.assertFalse(job_path.exists())
            self.assertEqual((raw_dir / "job.pgm").read_bytes(), raw_bytes)
            self.assertGreater(report["connected_wall_pixels"], 0)
            bundle = json.loads((root / "postprocess-outbox/job.json").read_text())
            self.assertEqual(bundle["robot_pose"], report["corrected_robot_pose"])
            self.assertEqual(len(bundle["coordinate_transform"]), 3)


if __name__ == "__main__":
    unittest.main()
