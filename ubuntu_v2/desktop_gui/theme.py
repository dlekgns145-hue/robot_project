"""Application-wide light visual theme for the desktop control center."""

APP_STYLESHEET = r"""
QWidget#AppRoot {
    background: #f3f6fa;
    color: #243447;
}

QWidget {
    color: #2b3a4a;
    font-family: "SF Pro Display", "Inter", "Segoe UI", sans-serif;
    font-size: 13px;
}

QFrame#HeaderPanel,
QFrame#PreviewCard {
    background: #ffffff;
    border: 1px solid #dbe3ec;
    border-radius: 14px;
}

QLabel#Eyebrow,
QLabel#SectionKicker,
QLabel#SubsectionLabel,
QLabel#MetricCaption {
    color: #1976d2;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}

QLabel#AppTitle {
    color: #172536;
    font-size: 25px;
    font-weight: 750;
}

QLabel#Subtitle {
    color: #6b7c8f;
    font-size: 12px;
}

QLabel#ConnectionStatus,
QLabel#SettingsConnectionStatus {
    padding: 9px 13px;
    border-radius: 16px;
    font-size: 12px;
    font-weight: 700;
}

QLabel#ConnectionStatus {
    min-width: 185px;
}

QLabel#ConnectionStatus[state="offline"],
QLabel#SettingsConnectionStatus[state="offline"] {
    color: #b42336;
    background: #fff0f1;
    border: 1px solid #f3c5ca;
}

QLabel#ConnectionStatus[state="waiting"],
QLabel#SettingsConnectionStatus[state="waiting"] {
    color: #9a5b00;
    background: #fff8e6;
    border: 1px solid #efd79a;
}

QLabel#ConnectionStatus[state="online"],
QLabel#SettingsConnectionStatus[state="online"] {
    color: #087a57;
    background: #eaf8f2;
    border: 1px solid #a9dfcc;
}

QPushButton#HeaderSettingsButton {
    min-width: 72px;
    min-height: 34px;
}

QLabel#CameraBadge {
    color: #0c6ba1;
    background: #eaf6ff;
    border: 1px solid #bfdef2;
    border-radius: 10px;
    padding: 4px 9px;
    font-size: 9px;
    font-weight: 700;
}

QLabel#VideoSurface {
    color: #a9b7c5;
    background: #16202a;
    border: 1px solid #263746;
    border-radius: 10px;
    font-size: 14px;
}

QLabel#VisionMetrics {
    color: #526579;
    background: #f3f7fb;
    border: 1px solid #dbe4ed;
    border-radius: 9px;
    padding: 4px 9px;
    font-family: "SF Mono", "JetBrains Mono", monospace;
    font-size: 11px;
}

QLabel#ControlHint,
QLabel#SettingsHint {
    color: #718397;
    padding-top: 4px;
    font-size: 10px;
}

QLabel#SettingsInfo {
    color: #34617f;
    background: #edf7fd;
    border: 1px solid #c9e4f4;
    border-radius: 8px;
    padding: 10px;
    font-size: 11px;
}

QGroupBox {
    background: #ffffff;
    border: 1px solid #dbe3ec;
    border-radius: 12px;
    margin-top: 13px;
    padding: 15px 12px 12px 12px;
    font-size: 13px;
    font-weight: 650;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 14px;
    padding: 0 7px;
    color: #26384a;
    background: #ffffff;
}

QFrame[card="metric"] {
    background: #f7f9fc;
    border: 1px solid #e0e7ef;
    border-radius: 9px;
}

QLabel#MetricValue {
    color: #1f3042;
    font-size: 13px;
    font-weight: 650;
}

QLineEdit,
QSpinBox,
QDoubleSpinBox,
QComboBox {
    min-height: 34px;
    padding: 0 10px;
    color: #243447;
    background: #f8fafc;
    border: 1px solid #cfd9e4;
    border-radius: 7px;
    selection-background-color: #2f80d0;
    selection-color: #ffffff;
}

QLineEdit:focus,
QSpinBox:focus,
QDoubleSpinBox:focus,
QComboBox:focus {
    border: 1px solid #2582d8;
    background: #ffffff;
}

QLineEdit:disabled,
QSpinBox:disabled,
QDoubleSpinBox:disabled,
QComboBox:disabled {
    color: #9ba8b5;
    background: #edf1f5;
    border-color: #dce3ea;
}

QComboBox::drop-down {
    width: 28px;
    border: 0;
}

QComboBox QAbstractItemView {
    color: #253547;
    background: #ffffff;
    border: 1px solid #cbd7e3;
    selection-background-color: #dceeff;
    selection-color: #174f82;
    outline: 0;
}

QSpinBox::up-button,
QDoubleSpinBox::up-button,
QSpinBox::down-button,
QDoubleSpinBox::down-button {
    width: 19px;
    background: #e9eff5;
    border: 0;
}

QPushButton {
    min-height: 34px;
    padding: 0 13px;
    color: #304255;
    background: #eef3f8;
    border: 1px solid #ced9e4;
    border-radius: 7px;
    font-weight: 650;
}

QPushButton:hover {
    background: #e1ebf4;
    border-color: #aec1d2;
}

QPushButton:pressed,
QPushButton:checked {
    color: #174f82;
    background: #d7eafe;
    border-color: #71afe3;
}

QPushButton:disabled {
    color: #9ba8b5;
    background: #edf1f4;
    border-color: #dce3e9;
}

QPushButton[role="primary"] {
    color: #ffffff;
    background: #1976d2;
    border-color: #1976d2;
}

QPushButton[role="primary"]:hover {
    background: #1268bd;
    border-color: #1268bd;
}

QPushButton[role="primary"]:checked {
    color: #0e4776;
    background: #b9ddfa;
    border-color: #5da6df;
}

QPushButton[role="secondary"] {
    color: #12608f;
    background: #e8f5fc;
    border-color: #aad5ec;
}

QPushButton[role="quiet"] {
    color: #53677a;
    background: #ffffff;
    border-color: #ccd8e3;
}

QPushButton[role="drive"],
QPushButton[role="driveStop"] {
    color: #2a4053;
    background: #f3f7fb;
    border: 1px solid #c7d6e3;
    font-size: 12px;
}

QPushButton[role="drive"]:pressed {
    color: #ffffff;
    background: #2582d8;
    border-color: #1268bd;
}

QPushButton[role="driveStop"] {
    color: #b42336;
    background: #fff1f2;
    border-color: #efc5ca;
}

QPushButton#EmergencyButton {
    color: #ffffff;
    background: #cf3044;
    border: 1px solid #bf2437;
    border-radius: 10px;
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 0.5px;
}

QPushButton#EmergencyButton:hover {
    background: #bb2639;
}

QPushButton#EmergencyButton:pressed {
    background: #8d1d2b;
}

QPlainTextEdit {
    color: #3f5569;
    background: #f7f9fb;
    border: 1px solid #d8e1e9;
    border-radius: 8px;
    padding: 8px;
    font-family: "SF Mono", "JetBrains Mono", monospace;
    font-size: 11px;
}

QScrollArea,
QWidget#ControlPanel,
QWidget#ControlPage,
QWidget#SettingsPage,
QWidget#SettingsContent,
QWidget#NavigationPage {
    background: transparent;
    border: 0;
}

QWidget#OperationsContent {
    background: #f8fafc;
}

QTabWidget#MainTabs::pane {
    background: #f8fafc;
    border: 1px solid #d9e2eb;
    border-radius: 12px;
    top: -1px;
}

QTabWidget#MainTabs QTabBar::tab {
    min-width: 112px;
    min-height: 34px;
    margin-right: 5px;
    padding: 0 14px;
    color: #65788b;
    background: #e9eff5;
    border: 1px solid #d5dfe8;
    border-bottom: 0;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    font-weight: 700;
}

QTabWidget#MainTabs QTabBar::tab:hover {
    color: #285c88;
    background: #deebf6;
}

QTabWidget#MainTabs QTabBar::tab:selected {
    color: #ffffff;
    background: #1976d2;
    border-color: #1976d2;
}

QLabel#PageTitle {
    color: #182a3c;
    font-size: 22px;
    font-weight: 750;
}

QLabel#OperationsSectionTitle {
    color: #26394b;
    font-size: 15px;
    font-weight: 700;
}

QLabel#OperationsHint,
QLabel#OperationsMetricLabel {
    color: #718397;
    font-size: 11px;
}

QLabel#OperationsMetricValue {
    color: #1b3044;
    font-size: 24px;
    font-weight: 800;
}

QLabel#OperationsSource {
    color: #087a57;
    background: #eaf8f2;
    border: 1px solid #a9dfcc;
    border-radius: 12px;
    padding: 6px 10px;
    font-size: 10px;
    font-weight: 700;
}

QLabel#OperationsSource[demo="true"] {
    color: #9a5b00;
    background: #fff8e6;
    border-color: #efd79a;
}

QFrame[card="operations"],
QFrame#CalendarDetail {
    background: #ffffff;
    border: 1px solid #dbe3ec;
    border-radius: 10px;
}

QLabel#CalendarTotal {
    color: #1565a8;
    background: #eaf5fd;
    border: 1px solid #bddcf1;
    border-radius: 8px;
    padding: 9px 12px;
    font-size: 14px;
    font-weight: 750;
}

QTableWidget {
    color: #304255;
    background: #ffffff;
    alternate-background-color: #f7f9fb;
    border: 1px solid #d8e1e9;
    border-radius: 8px;
    gridline-color: #e5ebf0;
    selection-background-color: #dbeeff;
    selection-color: #174f82;
}

QHeaderView::section {
    color: #52677a;
    background: #eef3f7;
    border: 0;
    border-right: 1px solid #d9e2ea;
    border-bottom: 1px solid #d9e2ea;
    padding: 8px;
    font-size: 11px;
    font-weight: 700;
}

QCalendarWidget QWidget {
    alternate-background-color: #f4f7fa;
}

QCalendarWidget QAbstractItemView:enabled {
    color: #304255;
    background: #ffffff;
    selection-background-color: #1976d2;
    selection-color: #ffffff;
    border: 1px solid #d8e1e9;
}

QCalendarWidget QToolButton {
    color: #304255;
    background: #eef3f7;
    border: 1px solid #ccd8e3;
    border-radius: 6px;
    margin: 3px;
    padding: 5px 9px;
}

QScrollBar:vertical {
    width: 8px;
    background: transparent;
    margin: 3px 0;
}

QScrollBar::handle:vertical {
    min-height: 32px;
    background: #b8c6d3;
    border-radius: 4px;
}

QScrollBar::handle:vertical:hover {
    background: #91a6b8;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical {
    height: 0;
    background: transparent;
}

QToolTip {
    color: #ffffff;
    background: #34495d;
    border: 1px solid #25384a;
    padding: 5px;
}
"""
