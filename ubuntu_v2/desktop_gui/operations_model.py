"""Pure helpers for presenting robot operations history."""

from __future__ import annotations

from datetime import date, datetime, time, timedelta
from typing import Any, Iterable


def parse_timestamp(value: object) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.astimezone()
    return parsed.astimezone()


def format_duration(seconds: float | int) -> str:
    total = max(0, int(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours}시간 {minutes:02d}분"
    if minutes:
        return f"{minutes}분 {seconds:02d}초"
    return f"{seconds}초"


def format_timestamp(value: object, *, empty: str = "-") -> str:
    parsed = parse_timestamp(value)
    return empty if parsed is None else parsed.strftime("%Y-%m-%d %H:%M:%S")


def session_interval(
    session: dict[str, Any], now: datetime | None = None
) -> tuple[datetime, datetime] | None:
    started = parse_timestamp(session.get("started_at"))
    ended = parse_timestamp(session.get("ended_at"))
    if started is None:
        return None
    current = (now or datetime.now().astimezone()).astimezone()
    return started, max(started, ended or current)


def duration_on_date(
    sessions: Iterable[dict[str, Any]],
    target: date,
    now: datetime | None = None,
) -> int:
    current = (now or datetime.now().astimezone()).astimezone()
    tzinfo = current.tzinfo
    day_start = datetime.combine(target, time.min, tzinfo=tzinfo)
    day_end = day_start + timedelta(days=1)
    total = 0.0
    for session in sessions:
        interval = session_interval(session, current)
        if interval is None:
            continue
        started, ended = interval
        overlap_start = max(started, day_start)
        overlap_end = min(ended, day_end)
        if overlap_end > overlap_start:
            total += (overlap_end - overlap_start).total_seconds()
    return max(0, int(total))


def active_dates(
    sessions: Iterable[dict[str, Any]], now: datetime | None = None
) -> set[date]:
    current = (now or datetime.now().astimezone()).astimezone()
    dates: set[date] = set()
    for session in sessions:
        interval = session_interval(session, current)
        if interval is None:
            continue
        cursor = interval[0].date()
        last = (interval[1] - timedelta(microseconds=1)).date()
        while cursor <= last:
            dates.add(cursor)
            cursor += timedelta(days=1)
    return dates


def monthly_duration(
    sessions: Iterable[dict[str, Any]],
    year: int,
    month: int,
    now: datetime | None = None,
) -> int:
    if month == 12:
        next_month = date(year + 1, 1, 1)
    else:
        next_month = date(year, month + 1, 1)
    cursor = date(year, month, 1)
    total = 0
    while cursor < next_month:
        total += duration_on_date(sessions, cursor, now)
        cursor += timedelta(days=1)
    return total


def summary(snapshot: dict[str, Any], now: datetime | None = None) -> dict[str, int]:
    current = (now or datetime.now().astimezone()).astimezone()
    robots = list(snapshot.get("robots") or [])
    sessions = list(snapshot.get("sessions") or [])
    online = sum(bool(robot.get("online")) for robot in robots)
    return {
        "online": online,
        "offline": max(0, len(robots) - online),
        "today_seconds": duration_on_date(sessions, current.date(), current),
        "month_seconds": monthly_duration(
            sessions, current.year, current.month, current
        ),
        "session_count": len(sessions),
    }
