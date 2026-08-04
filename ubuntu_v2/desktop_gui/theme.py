"""Application-wide visual theme for the desktop control center."""

APP_STYLESHEET = r"""
QWidget#AppRoot {
    background: #080d13;
    color: #e8eef5;
}

QWidget {
    color: #dce6f0;
    font-family: "SF Pro Display", "Inter", "Segoe UI", sans-serif;
    font-size: 13px;
}

QFrame#HeaderPanel,
QFrame#PreviewCard {
    background: #101821;
    border: 1px solid #21303d;
    border-radius: 14px;
}

QLabel#Eyebrow,
QLabel#SectionKicker,
QLabel#SubsectionLabel,
QLabel#MetricCaption {
    color: #5eb8ff;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 1px;
}

QLabel#AppTitle {
    color: #f6f9fc;
    font-size: 25px;
    font-weight: 700;
}

QLabel#Subtitle {
    color: #8193a5;
    font-size: 12px;
}

QLabel#ConnectionStatus {
    min-width: 185px;
    padding: 10px 14px;
    border-radius: 17px;
    font-size: 12px;
    font-weight: 700;
}

QLabel#ConnectionStatus[state="offline"] {
    color: #ff8b8b;
    background: #2a171b;
    border: 1px solid #573037;
}

QLabel#ConnectionStatus[state="waiting"] {
    color: #ffc86b;
    background: #2b2415;
    border: 1px solid #5a4923;
}

QLabel#ConnectionStatus[state="online"] {
    color: #58e0b5;
    background: #102a25;
    border: 1px solid #225548;
}

QLabel#CameraBadge {
    color: #78d4ff;
    background: #102c3b;
    border: 1px solid #204c61;
    border-radius: 10px;
    padding: 4px 9px;
    font-size: 9px;
    font-weight: 700;
}

QLabel#VideoSurface {
    color: #708396;
    background: #05080c;
    border: 1px solid #1e2a35;
    border-radius: 10px;
    font-size: 14px;
}

QLabel#VisionMetrics {
    color: #9aabba;
    background: #0b1219;
    border: 1px solid #1e303e;
    border-radius: 9px;
    padding: 4px 9px;
    font-family: "SF Mono", "JetBrains Mono", monospace;
    font-size: 11px;
}

QLabel#ControlHint {
    color: #70879a;
    padding-top: 4px;
    font-size: 10px;
}

QGroupBox {
    background: #101821;
    border: 1px solid #21303d;
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
    color: #edf4fb;
    background: #101821;
}

QFrame[card="metric"] {
    background: #0b1219;
    border: 1px solid #1d2a36;
    border-radius: 9px;
}

QLabel#MetricValue {
    color: #eef5fb;
    font-size: 13px;
    font-weight: 600;
}

QLineEdit,
QSpinBox,
QDoubleSpinBox,
QComboBox {
    min-height: 34px;
    padding: 0 10px;
    color: #e7eef5;
    background: #0a1118;
    border: 1px solid #293947;
    border-radius: 7px;
    selection-background-color: #2878c7;
}

QLineEdit:focus,
QSpinBox:focus,
QDoubleSpinBox:focus,
QComboBox:focus {
    border: 1px solid #4caeff;
    background: #0c151e;
}

QLineEdit:disabled,
QSpinBox:disabled,
QDoubleSpinBox:disabled,
QComboBox:disabled {
    color: #536474;
    background: #0a0f15;
    border-color: #18232d;
}

QComboBox::drop-down {
    width: 28px;
    border: 0;
}

QComboBox QAbstractItemView {
    color: #e7eef5;
    background: #111b25;
    border: 1px solid #2b3c4b;
    selection-background-color: #216aa7;
    outline: 0;
}

QSpinBox::up-button,
QDoubleSpinBox::up-button,
QSpinBox::down-button,
QDoubleSpinBox::down-button {
    width: 19px;
    background: #15222d;
    border: 0;
}

QPushButton {
    min-height: 34px;
    padding: 0 13px;
    color: #dbe7f1;
    background: #192633;
    border: 1px solid #2a3b4b;
    border-radius: 7px;
    font-weight: 650;
}

QPushButton:hover {
    background: #213446;
    border-color: #3b5870;
}

QPushButton:pressed,
QPushButton:checked {
    background: #274e70;
    border-color: #50adf5;
}

QPushButton:disabled {
    color: #596978;
    background: #111920;
    border-color: #1b2832;
}

QPushButton[role="primary"] {
    color: #f8fbff;
    background: #1976c9;
    border-color: #2f92e5;
}

QPushButton[role="primary"]:hover {
    background: #2286df;
}

QPushButton[role="primary"]:checked {
    color: #06223a;
    background: #64c4ff;
    border-color: #8bd4ff;
}

QPushButton[role="secondary"] {
    color: #8bd8ff;
    background: #132737;
    border-color: #28536e;
}

QPushButton[role="quiet"] {
    color: #aebdca;
    background: #131d26;
    border-color: #263541;
}

QPushButton[role="drive"],
QPushButton[role="driveStop"] {
    color: #dceaf5;
    background: #15232e;
    border: 1px solid #2b4355;
    font-size: 12px;
}

QPushButton[role="drive"]:pressed {
    color: #07131c;
    background: #55bfff;
    border-color: #8ed5ff;
}

QPushButton[role="driveStop"] {
    color: #ffb1b1;
    background: #2b1b20;
    border-color: #5a3039;
}

QPushButton#EmergencyButton {
    color: white;
    background: #c93443;
    border: 1px solid #ed5967;
    border-radius: 10px;
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 0.5px;
}

QPushButton#EmergencyButton:hover {
    background: #df3e4e;
}

QPushButton#EmergencyButton:pressed {
    background: #8f1f2b;
}

QPlainTextEdit {
    color: #95b0c7;
    background: #080e14;
    border: 1px solid #1c2a35;
    border-radius: 8px;
    padding: 8px;
    font-family: "SF Mono", "JetBrains Mono", monospace;
    font-size: 11px;
}

QScrollArea,
QWidget#ControlPanel {
    background: transparent;
    border: 0;
}

QScrollBar:vertical {
    width: 8px;
    background: transparent;
    margin: 3px 0;
}

QScrollBar::handle:vertical {
    min-height: 32px;
    background: #304352;
    border-radius: 4px;
}

QScrollBar::handle:vertical:hover {
    background: #426177;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical {
    height: 0;
    background: transparent;
}

QToolTip {
    color: #eaf2f8;
    background: #15222d;
    border: 1px solid #344b5d;
    padding: 5px;
}
"""
