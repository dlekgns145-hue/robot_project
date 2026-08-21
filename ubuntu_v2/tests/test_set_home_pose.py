from __future__ import annotations

import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from set_home_pose import home_set_payload  # noqa: E402


class SetHomePoseTests(unittest.TestCase):
    def test_payload_contains_command_and_connection_fields(self) -> None:
        self.assertEqual(
            home_set_payload("secret", "raspberrypi.local"),
            {
                "type": "home_set",
                "token": "secret",
                "robot_ip_hint": "raspberrypi.local",
            },
        )

    def test_empty_token_is_not_serialized(self) -> None:
        self.assertEqual(home_set_payload("", ""), {"type": "home_set"})


if __name__ == "__main__":
    unittest.main()
