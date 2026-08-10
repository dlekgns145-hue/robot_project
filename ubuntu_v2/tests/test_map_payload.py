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


if __name__ == "__main__":
    unittest.main()
