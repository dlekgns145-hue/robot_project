from __future__ import annotations

import base64
import json
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

APP_DIR = Path(__file__).resolve().parents[1] / "robot_app"
sys.path.insert(0, str(APP_DIR))

from map_inbox import MapInbox, decode_map_payload  # noqa: E402


def map_payload(image: bytes) -> dict[str, object]:
    return {
        "image_base64": base64.b64encode(zlib.compress(image)).decode("ascii"),
        "image_encoding": "zlib+base64",
        "width": 3,
        "height": 2,
        "resolution": 0.05,
        "origin_x": -1.0,
        "origin_y": 2.0,
        "origin_yaw": 0.0,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "robot_pose": {"x": 0.3, "y": -0.4, "yaw": 0.2},
    }


class MapInboxTests(unittest.TestCase):
    def test_completed_map_is_archived_before_atomic_job_is_published(self) -> None:
        image = b"P5\n3 2\n255\n" + bytes([0, 205, 254, 254, 0, 205])
        with tempfile.TemporaryDirectory() as directory:
            inbox = MapInbox(directory)
            job_id = inbox.stage(
                map_payload(image),
                {"state": "completed", "saved_map": "/maps/orchard_map", "save_sequence": 1},
            )

            self.assertIsNotNone(job_id)
            root = Path(directory)
            job = json.loads(
                (root / "postprocess-inbox" / f"{job_id}.json").read_text()
            )
            raw_prefix = root / job["input_prefix"]
            self.assertEqual(raw_prefix.with_suffix(".pgm").read_bytes(), image)
            self.assertIn("resolution: 0.05", raw_prefix.with_suffix(".yaml").read_text())
            self.assertEqual(job["robot_pose"], {"x": 0.3, "y": -0.4, "yaw": 0.2})

            self.assertIsNone(
                inbox.stage(
                    map_payload(image),
                    {"state": "completed", "saved_map": "same", "save_sequence": 2},
                )
            )
            self.assertEqual(
                len(list((root / "postprocess-inbox").glob("*.json"))), 1
            )

    def test_declared_dimensions_must_match_pgm(self) -> None:
        image = b"P5\n2 2\n255\n" + bytes([0, 0, 0, 0])
        with self.assertRaisesRegex(ValueError, "dimensions"):
            decode_map_payload(map_payload(image))


if __name__ == "__main__":
    unittest.main()
