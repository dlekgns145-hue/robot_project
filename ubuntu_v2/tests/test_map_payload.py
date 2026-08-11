from __future__ import annotations

import base64
import json
import os
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

APP_DIR = Path(__file__).resolve().parents[1] / "robot_app"
sys.path.insert(0, str(APP_DIR))

from map_payload import load_map_payload  # noqa: E402


class ServerMapPayloadTests(unittest.TestCase):
    @staticmethod
    def _write_map(root: Path, name: str, *, origin_x: float = 0.0) -> Path:
        image = root / f"{name}.pgm"
        image.write_bytes(b"P2\n2 2\n255\n254 0\n205 254\n")
        (root / f"{name}.yaml").write_text(
            f"resolution: 0.05\norigin: [{origin_x}, -2.0, 0.0]\n",
            encoding="utf-8",
        )
        return image

    def test_server_map_and_fresh_visual_layers_are_encoded(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / "orchard_map.pgm"
            image.write_bytes(b"P2\n2 2\n255\n254 0\n205 254\n")
            (root / "orchard_map.yaml").write_text(
                "resolution: 0.05\norigin: [1.0, -2.0, 0.0]\n",
                encoding="utf-8",
            )
            texture = root / "orchard_map_texture.png"
            obstacle = root / "orchard_map_obstacles.png"
            materials = root / "orchard_map_materials.json"
            texture.write_bytes(b"texture")
            obstacle.write_bytes(b"obstacle")
            materials.write_text(
                json.dumps({"dominant": "metal"}), encoding="utf-8"
            )
            newer = image.stat().st_mtime + 1.0
            for path in (texture, obstacle, materials):
                os.utime(path, (newer, newer))

            payload = load_map_payload(directory)

            self.assertEqual(payload["width"], 2)
            self.assertEqual(payload["height"], 2)
            self.assertEqual(payload["origin_x"], 1.0)
            self.assertEqual(
                zlib.decompress(base64.b64decode(payload["image_base64"])),
                image.read_bytes(),
            )
            self.assertEqual(
                base64.b64decode(payload["texture_base64"]), b"texture"
            )
            self.assertEqual(payload["obstacle_materials"]["dominant"], "metal")
            self.assertEqual(payload["map_source"], "saved")
            self.assertTrue(payload["navigation_safe"])

    def test_active_fresh_live_map_is_selected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_map(root, "orchard_map", origin_x=1.0)
            live_image = self._write_map(root, "orchard_map_live", origin_x=3.0)
            obstacle = root / "orchard_map_live_obstacles.png"
            obstacle.write_bytes(b"live-obstacle")
            os.utime(obstacle, (live_image.stat().st_mtime + 1.0,) * 2)
            (root / "orchard_map_live_status.json").write_text(
                json.dumps(
                    {
                        "active": True,
                        "updated_unix": 100.0,
                        "map_sequence": 42,
                        "occupancy_source": "lidar_slam_only",
                    }
                ),
                encoding="utf-8",
            )

            payload = load_map_payload(directory, now_unix=102.0)

            self.assertEqual(payload["map_source"], "live")
            self.assertEqual(payload["origin_x"], 3.0)
            self.assertEqual(payload["layer_revision"], 42)
            self.assertFalse(payload["visual_layer_navigation_safe"])
            self.assertEqual(
                base64.b64decode(payload["obstacle_texture_base64"]),
                b"live-obstacle",
            )

    def test_stale_or_inactive_live_map_falls_back_to_saved_map(self) -> None:
        for active, updated in ((True, 80.0), (False, 100.0)):
            with self.subTest(active=active, updated=updated):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    self._write_map(root, "orchard_map", origin_x=1.0)
                    self._write_map(root, "orchard_map_live", origin_x=3.0)
                    (root / "orchard_map_live_status.json").write_text(
                        json.dumps(
                            {"active": active, "updated_unix": updated}
                        ),
                        encoding="utf-8",
                    )

                    payload = load_map_payload(directory, now_unix=101.0)

                    self.assertEqual(payload["map_source"], "saved")
                    self.assertEqual(payload["origin_x"], 1.0)


if __name__ == "__main__":
    unittest.main()
