from __future__ import annotations

import sys
import tempfile
import unittest
from datetime import date, datetime, timedelta, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_app"))
sys.path.insert(0, str(ROOT / "desktop_gui"))

from operations_model import duration_on_date, format_duration  # noqa: E402
from operations_store import OperationsStore  # noqa: E402


KST = timezone(timedelta(hours=9))


class OperationsStoreTests(unittest.TestCase):
    def test_online_offline_transition_creates_work_session(self) -> None:
        current = [datetime(2026, 8, 5, 9, 0, tzinfo=KST)]
        with tempfile.TemporaryDirectory() as directory:
            store = OperationsStore(
                Path(directory) / "operations.sqlite3",
                robot_id="robot-1",
                robot_name="Robot 1",
                clock=lambda: current[0],
            )
            self.assertTrue(store.set_robot_state(True, ip="172.30.1.18"))
            current[0] += timedelta(hours=2, minutes=15)
            self.assertFalse(store.set_robot_state(True, ip="172.30.1.18"))
            current[0] += timedelta(minutes=45)
            self.assertTrue(store.set_robot_state(False, detail="power off"))

            snapshot = store.snapshot()
            store.close()

        self.assertFalse(snapshot["robots"][0]["online"])
        self.assertEqual(len(snapshot["sessions"]), 1)
        self.assertEqual(snapshot["sessions"][0]["duration_seconds"], 10800)
        self.assertEqual(
            [event["event_type"] for event in reversed(snapshot["events"])],
            ["online", "offline"],
        )

    def test_gateway_restart_closes_stale_open_session_at_last_seen(self) -> None:
        current = [datetime(2026, 8, 5, 9, 0, tzinfo=KST)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "operations.sqlite3"
            store = OperationsStore(
                path,
                robot_id="robot-1",
                robot_name="Robot 1",
                clock=lambda: current[0],
            )
            store.set_robot_state(True, ip="172.30.1.18")
            current[0] += timedelta(minutes=30)
            store.set_robot_state(True, ip="172.30.1.18")
            store.close()

            current[0] += timedelta(hours=1)
            recovered = OperationsStore(
                path,
                robot_id="robot-1",
                robot_name="Robot 1",
                clock=lambda: current[0],
            )
            recovered.mark_gateway_started()
            snapshot = recovered.snapshot()
            recovered.close()

        self.assertFalse(snapshot["robots"][0]["online"])
        self.assertEqual(snapshot["sessions"][0]["duration_seconds"], 1800)
        self.assertEqual(snapshot["sessions"][0]["end_reason"], "gateway restarted")


class OperationsModelTests(unittest.TestCase):
    def test_session_duration_is_split_across_calendar_days(self) -> None:
        sessions = [
            {
                "started_at": "2026-08-05T23:30:00+09:00",
                "ended_at": "2026-08-06T01:00:00+09:00",
            }
        ]
        self.assertEqual(duration_on_date(sessions, date(2026, 8, 5)), 1800)
        self.assertEqual(duration_on_date(sessions, date(2026, 8, 6)), 3600)

    def test_duration_format(self) -> None:
        self.assertEqual(format_duration(3661), "1시간 01분")
        self.assertEqual(format_duration(61), "1분 01초")


if __name__ == "__main__":
    unittest.main()

