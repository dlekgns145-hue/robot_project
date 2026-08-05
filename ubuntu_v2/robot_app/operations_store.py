"""Persistent robot availability events and work sessions."""

from __future__ import annotations

import sqlite3
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Callable


def local_now() -> datetime:
    return datetime.now().astimezone()


def iso_timestamp(value: datetime) -> str:
    if value.tzinfo is None:
        value = value.astimezone()
    return value.isoformat(timespec="seconds")


class OperationsStore:
    """Thread-safe SQLite store owned by the always-on Ubuntu gateway."""

    def __init__(
        self,
        path: str | Path,
        *,
        robot_id: str,
        robot_name: str,
        clock: Callable[[], datetime] = local_now,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.robot_id = robot_id.strip() or "robot-1"
        self.robot_name = robot_name.strip() or self.robot_id
        self.clock = clock
        self.lock = threading.RLock()
        self.connection = sqlite3.connect(self.path, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        with self.lock:
            self.connection.execute("PRAGMA journal_mode=WAL")
            self.connection.execute("PRAGMA foreign_keys=ON")
            self._create_schema()

    def _create_schema(self) -> None:
        self.connection.executescript(
            """
            CREATE TABLE IF NOT EXISTS robot_state (
                robot_id TEXT PRIMARY KEY,
                robot_name TEXT NOT NULL,
                online INTEGER NOT NULL DEFAULT 0,
                ip TEXT,
                last_changed_at TEXT NOT NULL,
                last_seen_at TEXT NOT NULL,
                active_session_id INTEGER
            );

            CREATE TABLE IF NOT EXISTS robot_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                robot_id TEXT NOT NULL,
                robot_name TEXT NOT NULL,
                event_type TEXT NOT NULL CHECK(event_type IN ('online', 'offline')),
                occurred_at TEXT NOT NULL,
                ip TEXT,
                detail TEXT
            );

            CREATE TABLE IF NOT EXISTS work_sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                robot_id TEXT NOT NULL,
                robot_name TEXT NOT NULL,
                started_at TEXT NOT NULL,
                ended_at TEXT,
                last_seen_at TEXT NOT NULL,
                start_ip TEXT,
                end_reason TEXT
            );

            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_events_time
                ON robot_events(occurred_at DESC);
            CREATE INDEX IF NOT EXISTS idx_sessions_start
                ON work_sessions(started_at DESC);
            """
        )
        self.connection.execute(
            "INSERT OR IGNORE INTO metadata(key, value) VALUES('revision', '0')"
        )
        self.connection.commit()

    def _revision(self) -> int:
        row = self.connection.execute(
            "SELECT value FROM metadata WHERE key='revision'"
        ).fetchone()
        return int(row["value"]) if row else 0

    def revision(self) -> int:
        with self.lock:
            return self._revision()

    def _bump_revision(self) -> int:
        revision = self._revision() + 1
        self.connection.execute(
            "UPDATE metadata SET value=? WHERE key='revision'", (str(revision),)
        )
        return revision

    def mark_gateway_started(self) -> None:
        """Close stale open sessions after an unclean gateway shutdown."""
        timestamp = iso_timestamp(self.clock())
        with self.lock:
            rows = self.connection.execute(
                "SELECT * FROM robot_state WHERE online=1"
            ).fetchall()
            if not rows:
                return
            for row in rows:
                ended_at = row["last_seen_at"] or timestamp
                session_id = row["active_session_id"]
                if session_id is not None:
                    self.connection.execute(
                        """
                        UPDATE work_sessions
                        SET ended_at=?, last_seen_at=?, end_reason=?
                        WHERE id=? AND ended_at IS NULL
                        """,
                        (ended_at, ended_at, "gateway restarted", session_id),
                    )
                self.connection.execute(
                    """
                    INSERT INTO robot_events(
                        robot_id, robot_name, event_type, occurred_at, ip, detail
                    ) VALUES(?, ?, 'offline', ?, ?, ?)
                    """,
                    (
                        row["robot_id"],
                        row["robot_name"],
                        ended_at,
                        row["ip"],
                        "게이트웨이 재시작으로 이전 세션 종료",
                    ),
                )
                self.connection.execute(
                    """
                    UPDATE robot_state
                    SET online=0, active_session_id=NULL,
                        last_changed_at=?, last_seen_at=?
                    WHERE robot_id=?
                    """,
                    (ended_at, ended_at, row["robot_id"]),
                )
            self._bump_revision()
            self.connection.commit()

    def set_robot_state(
        self,
        online: bool,
        *,
        ip: str = "",
        detail: str = "",
        observed_at: datetime | None = None,
    ) -> bool:
        """Record an online/offline transition. Returns True on a transition."""
        timestamp = iso_timestamp(observed_at or self.clock())
        online_value = int(bool(online))
        with self.lock:
            state = self.connection.execute(
                "SELECT * FROM robot_state WHERE robot_id=?", (self.robot_id,)
            ).fetchone()
            if state is None:
                self.connection.execute(
                    """
                    INSERT INTO robot_state(
                        robot_id, robot_name, online, ip,
                        last_changed_at, last_seen_at, active_session_id
                    ) VALUES(?, ?, 0, ?, ?, ?, NULL)
                    """,
                    (
                        self.robot_id,
                        self.robot_name,
                        ip or None,
                        timestamp,
                        timestamp,
                    ),
                )
                state = self.connection.execute(
                    "SELECT * FROM robot_state WHERE robot_id=?", (self.robot_id,)
                ).fetchone()

            assert state is not None
            if int(state["online"]) == online_value:
                self.connection.execute(
                    """
                    UPDATE robot_state
                    SET robot_name=?, ip=COALESCE(NULLIF(?, ''), ip), last_seen_at=?
                    WHERE robot_id=?
                    """,
                    (self.robot_name, ip, timestamp, self.robot_id),
                )
                if online and state["active_session_id"] is not None:
                    self.connection.execute(
                        "UPDATE work_sessions SET last_seen_at=? WHERE id=?",
                        (timestamp, state["active_session_id"]),
                    )
                self.connection.commit()
                return False

            self.connection.execute(
                """
                INSERT INTO robot_events(
                    robot_id, robot_name, event_type, occurred_at, ip, detail
                ) VALUES(?, ?, ?, ?, ?, ?)
                """,
                (
                    self.robot_id,
                    self.robot_name,
                    "online" if online else "offline",
                    timestamp,
                    ip or state["ip"],
                    detail or None,
                ),
            )

            active_session_id: int | None = None
            if online:
                cursor = self.connection.execute(
                    """
                    INSERT INTO work_sessions(
                        robot_id, robot_name, started_at, ended_at,
                        last_seen_at, start_ip, end_reason
                    ) VALUES(?, ?, ?, NULL, ?, ?, NULL)
                    """,
                    (
                        self.robot_id,
                        self.robot_name,
                        timestamp,
                        timestamp,
                        ip or None,
                    ),
                )
                active_session_id = int(cursor.lastrowid)
            else:
                active_session_id = state["active_session_id"]
                if active_session_id is not None:
                    self.connection.execute(
                        """
                        UPDATE work_sessions
                        SET ended_at=?, last_seen_at=?, end_reason=?
                        WHERE id=? AND ended_at IS NULL
                        """,
                        (timestamp, timestamp, detail or "robot offline", active_session_id),
                    )
                active_session_id = None

            self.connection.execute(
                """
                UPDATE robot_state
                SET robot_name=?, online=?, ip=COALESCE(NULLIF(?, ''), ip),
                    last_changed_at=?, last_seen_at=?, active_session_id=?
                WHERE robot_id=?
                """,
                (
                    self.robot_name,
                    online_value,
                    ip,
                    timestamp,
                    timestamp,
                    active_session_id,
                    self.robot_id,
                ),
            )
            self._bump_revision()
            self.connection.commit()
            return True

    def snapshot(self, *, event_limit: int = 100, session_limit: int = 500) -> dict[str, Any]:
        now = self.clock()
        now_iso = iso_timestamp(now)
        with self.lock:
            robots = [dict(row) for row in self.connection.execute(
                "SELECT * FROM robot_state ORDER BY robot_name"
            ).fetchall()]
            events = [dict(row) for row in self.connection.execute(
                "SELECT * FROM robot_events ORDER BY occurred_at DESC LIMIT ?",
                (event_limit,),
            ).fetchall()]
            sessions = [dict(row) for row in self.connection.execute(
                "SELECT * FROM work_sessions ORDER BY started_at DESC LIMIT ?",
                (session_limit,),
            ).fetchall()]
            revision = self._revision()

        for robot in robots:
            robot["online"] = bool(robot["online"])
            session_id = robot.pop("active_session_id", None)
            robot["active_session_started_at"] = None
            if session_id is not None:
                matching = next(
                    (session for session in sessions if session["id"] == session_id),
                    None,
                )
                if matching is not None:
                    robot["active_session_started_at"] = matching["started_at"]

        for session in sessions:
            end_text = session["ended_at"] or now_iso
            started = datetime.fromisoformat(session["started_at"])
            ended = datetime.fromisoformat(end_text)
            session["duration_seconds"] = max(
                0, int((ended - started).total_seconds())
            )
            session["active"] = session["ended_at"] is None

        return {
            "revision": revision,
            "generated_at": now_iso,
            "robots": robots,
            "events": events,
            "sessions": sessions,
        }

    def close(self) -> None:
        with self.lock:
            self.connection.close()
