"""Administrator pages for fleet status, work logs, and calendar history."""

from __future__ import annotations

from datetime import date, datetime, timedelta
from typing import Any

from PySide6.QtCore import QDate, QTimer, Signal
from PySide6.QtGui import QColor, QTextCharFormat
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCalendarWidget,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from operations_model import (
    active_dates,
    duration_on_date,
    format_duration,
    format_timestamp,
    parse_timestamp,
    session_interval,
    summary,
)


def _table(headers: list[str], minimum_height: int) -> QTableWidget:
    widget = QTableWidget(0, len(headers))
    widget.setHorizontalHeaderLabels(headers)
    widget.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
    widget.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
    widget.setAlternatingRowColors(True)
    widget.setMinimumHeight(minimum_height)
    widget.verticalHeader().setVisible(False)
    widget.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    return widget


def _item(text: object, *, color: str | None = None) -> QTableWidgetItem:
    item = QTableWidgetItem(str(text))
    if color:
        item.setForeground(QColor(color))
    return item


def _section_title(title: str, subtitle: str = "") -> QWidget:
    panel = QWidget()
    layout = QHBoxLayout(panel)
    layout.setContentsMargins(0, 4, 0, 3)
    label = QLabel(title)
    label.setObjectName("OperationsSectionTitle")
    layout.addWidget(label)
    if subtitle:
        detail = QLabel(subtitle)
        detail.setObjectName("OperationsHint")
        layout.addWidget(detail)
    layout.addStretch(1)
    return panel


class OperationsDashboardPage(QScrollArea):
    demo_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.snapshot: dict[str, Any] = {
            "robots": [],
            "sessions": [],
            "events": [],
        }

        content = QWidget()
        content.setObjectName("OperationsContent")
        self.setWidget(content)
        root = QVBoxLayout(content)
        root.setContentsMargins(6, 8, 10, 14)
        root.setSpacing(13)

        heading = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("로봇 운영 현황")
        title.setObjectName("PageTitle")
        subtitle = QLabel(
            "게이트웨이가 감지한 온라인 상태와 작업 세션을 영구 기록합니다."
        )
        subtitle.setObjectName("Subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        heading.addLayout(title_box)
        heading.addStretch(1)
        self.source_label = QLabel("서버 기록 대기 중")
        self.source_label.setObjectName("OperationsSource")
        heading.addWidget(self.source_label)
        demo_button = QPushButton("샘플 데이터 미리보기")
        demo_button.setProperty("role", "quiet")
        demo_button.clicked.connect(self.demo_requested)
        heading.addWidget(demo_button)
        root.addLayout(heading)

        metrics = QGridLayout()
        metrics.setSpacing(10)
        self.online_value = QLabel("0")
        self.offline_value = QLabel("0")
        self.today_value = QLabel("0초")
        self.month_value = QLabel("0초")
        metric_values = (
            ("ONLINE", "현재 온라인", self.online_value),
            ("OFFLINE", "현재 오프라인", self.offline_value),
            ("TODAY", "오늘 작업시간", self.today_value),
            ("MONTH", "이번 달 작업시간", self.month_value),
        )
        for column, (kicker, label, value) in enumerate(metric_values):
            card = QFrame()
            card.setProperty("card", "operations")
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(16, 13, 16, 14)
            caption = QLabel(kicker)
            caption.setObjectName("MetricCaption")
            description = QLabel(label)
            description.setObjectName("OperationsMetricLabel")
            value.setObjectName("OperationsMetricValue")
            card_layout.addWidget(caption)
            card_layout.addWidget(value)
            card_layout.addWidget(description)
            metrics.addWidget(card, 0, column)
        root.addLayout(metrics)

        root.addWidget(_section_title("로봇 상태", "마지막 연결 감지 기준"))
        self.robot_table = _table(
            ["로봇", "상태", "IP 주소", "상태 변경", "현재 작업시간"], 145
        )
        root.addWidget(self.robot_table)

        root.addWidget(_section_title("작업 로그", "온라인부터 오프라인까지 한 세션"))
        self.session_table = _table(
            ["로봇", "켜짐/온라인", "꺼짐/오프라인", "작업시간", "종료 사유"],
            210,
        )
        root.addWidget(self.session_table)

        root.addWidget(_section_title("최근 상태 이벤트"))
        self.event_table = _table(
            ["감지 시각", "이벤트", "로봇", "IP 주소", "상세"], 170
        )
        root.addWidget(self.event_table)
        root.addStretch(1)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_live_values)
        self.refresh_timer.start(1000)
        self.update_snapshot(self.snapshot)

    def update_snapshot(self, snapshot: dict[str, Any], *, demo: bool = False) -> None:
        self.snapshot = snapshot
        self.source_label.setText("샘플 데이터" if demo else "Ubuntu 서버 기록")
        self.source_label.setProperty("demo", demo)
        style = self.source_label.style()
        style.unpolish(self.source_label)
        style.polish(self.source_label)
        self._refresh_live_values()
        self._render_robots()
        self._render_sessions()
        self._render_events()

    def _refresh_live_values(self) -> None:
        values = summary(self.snapshot)
        self.online_value.setText(str(values["online"]))
        self.offline_value.setText(str(values["offline"]))
        self.today_value.setText(format_duration(values["today_seconds"]))
        self.month_value.setText(format_duration(values["month_seconds"]))
        if any(session.get("active") for session in self.snapshot.get("sessions", [])):
            self._render_sessions()
            self._render_robots()

    def _render_robots(self) -> None:
        robots = list(self.snapshot.get("robots") or [])
        self.robot_table.setRowCount(len(robots))
        now = datetime.now().astimezone()
        for row, robot in enumerate(robots):
            online = bool(robot.get("online"))
            status_text = "● 온라인" if online else "● 오프라인"
            status_color = "#58e0b5" if online else "#ff8b8b"
            started = parse_timestamp(robot.get("active_session_started_at"))
            elapsed = "-"
            if online and started is not None:
                elapsed = format_duration((now - started).total_seconds())
            values = (
                _item(robot.get("robot_name") or robot.get("robot_id") or "-"),
                _item(status_text, color=status_color),
                _item(robot.get("ip") or "-"),
                _item(format_timestamp(robot.get("last_changed_at"))),
                _item(elapsed),
            )
            for column, item in enumerate(values):
                self.robot_table.setItem(row, column, item)

    def _render_sessions(self) -> None:
        sessions = list(self.snapshot.get("sessions") or [])[:100]
        self.session_table.setRowCount(len(sessions))
        now = datetime.now().astimezone()
        for row, session in enumerate(sessions):
            interval = session_interval(session, now)
            duration = 0 if interval is None else int(
                (interval[1] - interval[0]).total_seconds()
            )
            active = bool(session.get("active")) or not session.get("ended_at")
            values = (
                session.get("robot_name") or session.get("robot_id") or "-",
                format_timestamp(session.get("started_at")),
                "작업 중" if active else format_timestamp(session.get("ended_at")),
                format_duration(duration),
                "온라인" if active else session.get("end_reason") or "-",
            )
            for column, value in enumerate(values):
                color = "#58e0b5" if active and column == 2 else None
                self.session_table.setItem(row, column, _item(value, color=color))

    def _render_events(self) -> None:
        events = list(self.snapshot.get("events") or [])[:100]
        self.event_table.setRowCount(len(events))
        for row, event in enumerate(events):
            online = event.get("event_type") == "online"
            values = (
                format_timestamp(event.get("occurred_at")),
                "켜짐 · 온라인" if online else "꺼짐 · 오프라인",
                event.get("robot_name") or event.get("robot_id") or "-",
                event.get("ip") or "-",
                event.get("detail") or "-",
            )
            for column, value in enumerate(values):
                color = (
                    "#58e0b5" if online else "#ff8b8b"
                ) if column == 1 else None
                self.event_table.setItem(row, column, _item(value, color=color))


class WorkCalendarPage(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.snapshot: dict[str, Any] = {"sessions": [], "events": []}
        self.highlighted_dates: set[date] = set()

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 10, 8, 12)
        title = QLabel("작업 캘린더")
        title.setObjectName("PageTitle")
        subtitle = QLabel("날짜를 선택하면 해당 날짜의 작업 세션과 누적 시간을 표시합니다.")
        subtitle.setObjectName("Subtitle")
        root.addWidget(title)
        root.addWidget(subtitle)

        body = QHBoxLayout()
        body.setSpacing(16)
        self.calendar = QCalendarWidget()
        self.calendar.setGridVisible(True)
        self.calendar.setVerticalHeaderFormat(
            QCalendarWidget.VerticalHeaderFormat.NoVerticalHeader
        )
        self.calendar.selectionChanged.connect(self._render_selected_date)
        body.addWidget(self.calendar, 1)

        detail_panel = QFrame()
        detail_panel.setObjectName("CalendarDetail")
        detail = QVBoxLayout(detail_panel)
        self.selected_title = QLabel()
        self.selected_title.setObjectName("OperationsSectionTitle")
        self.selected_total = QLabel("총 작업시간 0초")
        self.selected_total.setObjectName("CalendarTotal")
        detail.addWidget(self.selected_title)
        detail.addWidget(self.selected_total)
        self.day_sessions = _table(
            ["로봇", "시작", "종료", "해당 날짜 작업시간"], 280
        )
        detail.addWidget(self.day_sessions, 1)
        self.day_events = _table(["시각", "상태", "상세"], 145)
        detail.addWidget(self.day_events)
        body.addWidget(detail_panel, 2)
        root.addLayout(body, 1)
        self._render_selected_date()

    def update_snapshot(self, snapshot: dict[str, Any]) -> None:
        self.snapshot = snapshot
        self._update_highlights()
        self._render_selected_date()

    def _selected_python_date(self) -> date:
        selected = self.calendar.selectedDate()
        return date(selected.year(), selected.month(), selected.day())

    def _update_highlights(self) -> None:
        clear_format = QTextCharFormat()
        for highlighted in self.highlighted_dates:
            self.calendar.setDateTextFormat(
                QDate(highlighted.year, highlighted.month, highlighted.day),
                clear_format,
            )
        self.highlighted_dates = active_dates(self.snapshot.get("sessions") or [])
        active_format = QTextCharFormat()
        active_format.setBackground(QColor("#174d68"))
        active_format.setForeground(QColor("#eaf8ff"))
        active_format.setFontWeight(700)
        for highlighted in self.highlighted_dates:
            self.calendar.setDateTextFormat(
                QDate(highlighted.year, highlighted.month, highlighted.day),
                active_format,
            )

    def _render_selected_date(self) -> None:
        selected = self._selected_python_date()
        sessions = list(self.snapshot.get("sessions") or [])
        events = list(self.snapshot.get("events") or [])
        now = datetime.now().astimezone()
        self.selected_title.setText(selected.strftime("%Y년 %m월 %d일"))
        total = duration_on_date(sessions, selected, now)
        self.selected_total.setText(f"총 작업시간  {format_duration(total)}")

        matching_sessions: list[tuple[dict[str, Any], int]] = []
        for session in sessions:
            duration = duration_on_date([session], selected, now)
            if duration > 0:
                matching_sessions.append((session, duration))
        self.day_sessions.setRowCount(len(matching_sessions))
        for row, (session, duration) in enumerate(matching_sessions):
            values = (
                session.get("robot_name") or session.get("robot_id") or "-",
                format_timestamp(session.get("started_at")),
                "작업 중"
                if not session.get("ended_at")
                else format_timestamp(session.get("ended_at")),
                format_duration(duration),
            )
            for column, value in enumerate(values):
                self.day_sessions.setItem(row, column, _item(value))

        matching_events = [
            event
            for event in events
            if (parsed := parse_timestamp(event.get("occurred_at"))) is not None
            and parsed.date() == selected
        ]
        self.day_events.setRowCount(len(matching_events))
        for row, event in enumerate(matching_events):
            parsed = parse_timestamp(event.get("occurred_at"))
            online = event.get("event_type") == "online"
            values = (
                "-" if parsed is None else parsed.strftime("%H:%M:%S"),
                "온라인" if online else "오프라인",
                event.get("detail") or "-",
            )
            for column, value in enumerate(values):
                color = (
                    "#58e0b5" if online else "#ff8b8b"
                ) if column == 1 else None
                self.day_events.setItem(row, column, _item(value, color=color))


def demo_snapshot() -> dict[str, Any]:
    now = datetime.now().astimezone().replace(microsecond=0)
    robot_id = "demo-robot-01"
    robot_name = "Yahboom Pi5 Robot 1"

    def stamp(value: datetime) -> str:
        return value.isoformat(timespec="seconds")

    sessions = []
    events = []
    for days_ago, start_hour, duration_hours in ((4, 9, 3), (2, 13, 5), (1, 10, 2)):
        started = (now - timedelta(days=days_ago)).replace(
            hour=start_hour, minute=0, second=0
        )
        ended = started + timedelta(hours=duration_hours, minutes=20)
        sessions.append(
            {
                "id": len(sessions) + 1,
                "robot_id": robot_id,
                "robot_name": robot_name,
                "started_at": stamp(started),
                "ended_at": stamp(ended),
                "last_seen_at": stamp(ended),
                "start_ip": "172.30.1.18",
                "end_reason": "정상 종료",
                "duration_seconds": int((ended - started).total_seconds()),
                "active": False,
            }
        )
        events.extend(
            [
                {
                    "id": len(events) + 1,
                    "robot_id": robot_id,
                    "robot_name": robot_name,
                    "event_type": "online",
                    "occurred_at": stamp(started),
                    "ip": "172.30.1.18",
                    "detail": "샘플 연결 감지",
                },
                {
                    "id": len(events) + 2,
                    "robot_id": robot_id,
                    "robot_name": robot_name,
                    "event_type": "offline",
                    "occurred_at": stamp(ended),
                    "ip": "172.30.1.18",
                    "detail": "샘플 정상 종료",
                },
            ]
        )

    active_start = now - timedelta(hours=1, minutes=12)
    sessions.insert(
        0,
        {
            "id": 99,
            "robot_id": robot_id,
            "robot_name": robot_name,
            "started_at": stamp(active_start),
            "ended_at": None,
            "last_seen_at": stamp(now),
            "start_ip": "172.30.1.18",
            "end_reason": None,
            "duration_seconds": int((now - active_start).total_seconds()),
            "active": True,
        },
    )
    events.insert(
        0,
        {
            "id": 99,
            "robot_id": robot_id,
            "robot_name": robot_name,
            "event_type": "online",
            "occurred_at": stamp(active_start),
            "ip": "172.30.1.18",
            "detail": "샘플 작업 시작",
        },
    )
    return {
        "revision": -1,
        "generated_at": stamp(now),
        "robots": [
            {
                "robot_id": robot_id,
                "robot_name": robot_name,
                "online": True,
                "ip": "172.30.1.18",
                "last_changed_at": stamp(active_start),
                "last_seen_at": stamp(now),
                "active_session_started_at": stamp(active_start),
            }
        ],
        "sessions": sessions,
        "events": events,
    }
