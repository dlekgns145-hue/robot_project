from __future__ import annotations

import base64
import hashlib
import json
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

ROBOT_DIR = Path(__file__).resolve().parents[2] / "robot_docker"
sys.path.insert(0, str(ROBOT_DIR))

from map_bundle import (  # noqa: E402
    active_navigation_paths,
    install_navigation_bundle,
    validate_navigation_bundle,
)


def corrected_bundle() -> dict[str, object]:
    image = b"P5\n3 2\n255\n" + bytes([0, 205, 254, 254, 0, 205])
    return {
        "job_id": "map-job-1",
        "navigation_safe": True,
        "image_base64": base64.b64encode(zlib.compress(image)).decode(),
        "image_encoding": "zlib+base64",
        "corrected_sha256": hashlib.sha256(image).hexdigest(),
        "source_sha256": "a" * 64,
        "width": 3,
        "height": 2,
        "resolution": 0.05,
        "origin_x": -1.0,
        "origin_y": -2.0,
        "origin_yaw": 0.0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "robot_pose": {"x": 0.25, "y": -0.5, "yaw": 0.4},
        "coordinate_transform": [
            [1.0, 0.0, 0.1],
            [0.0, 1.0, -0.2],
            [0.0, 0.0, 1.0],
        ],
    }


class RobotNavigationBundleTests(unittest.TestCase):
    def test_bundle_installs_versioned_map_and_pose_behind_atomic_link(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            manifest = install_navigation_bundle(corrected_bundle(), directory)
            paths = active_navigation_paths(directory)

            self.assertEqual(manifest["job_id"], "map-job-1")
            self.assertIsNotNone(paths)
            assert paths is not None
            self.assertEqual(paths[0].resolve().parent.name, "map-job-1")
            self.assertEqual(
                json.loads(paths[1].read_text()),
                {"x": 0.25, "y": -0.5, "yaw": 0.4},
            )
            self.assertTrue((Path(directory) / "orchard_map.pgm").exists() is False)
            # Re-delivery is idempotent and keeps the same bundle active.
            install_navigation_bundle(corrected_bundle(), directory)
            self.assertEqual(
                (Path(directory) / "navigation-current").resolve().name,
                "map-job-1",
            )

    def test_checksum_mismatch_is_rejected_before_any_install(self) -> None:
        bundle = corrected_bundle()
        bundle["corrected_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "checksum"):
            validate_navigation_bundle(bundle)


if __name__ == "__main__":
    unittest.main()
