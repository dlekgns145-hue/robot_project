"""Friendly visual theme for the end-user robot controller."""

USER_STYLESHEET = r"""
QWidget#AppRoot {
    background: #f4f9ee;
    color: #1c2b16;
}

QWidget {
    color: #263a1e;
    font-family: "Pretendard", "Apple SD Gothic Neo", "Segoe UI", sans-serif;
    font-size: 14px;
}

QFrame#Header,
QFrame#CameraCard,
QFrame#StatusCard {
    background: #ffffff;
    border: 1px solid #dce8cc;
    border-radius: 18px;
}

QLabel#Eyebrow {
    color: #558b2f;
    font-size: 11px;
    font-weight: 800;
    letter-spacing: 1px;
}

QLabel#Title {
    color: #1b2c14;
    font-size: 27px;
    font-weight: 800;
}

QLabel#Subtitle,
QLabel#Hint,
QLabel#StatusCaption {
    color: #77896d;
    font-size: 12px;
}

QLabel#ConnectionStatus {
    min-width: 210px;
    padding: 11px 16px;
    border-radius: 18px;
    font-size: 12px;
    font-weight: 750;
}

QLabel#ConnectionStatus[state="offline"] {
    color: #b43845;
    background: #fff0f1;
    border: 1px solid #f2c7cb;
}

QLabel#ConnectionStatus[state="waiting"] {
    color: #9a6817;
    background: #fff8e6;
    border: 1px solid #eedcae;
}

QLabel#ConnectionStatus[state="online"] {
    color: #4c7a25;
    background: #eef7e0;
    border: 1px solid #cde3b3;
}

QLabel#VideoSurface {
    color: #8795a8;
    background: #0c1420;
    border: 1px solid #263448;
    border-radius: 14px;
    font-size: 15px;
}

QLabel#CameraMetrics {
    color: #587147;
    background: #eef5e2;
    border-radius: 10px;
    padding: 5px 10px;
    font-size: 11px;
}

QLabel#StatusValue {
    color: #1c2c15;
    font-size: 15px;
    font-weight: 750;
}

QGroupBox {
    background: #ffffff;
    border: 1px solid #dce8cc;
    border-radius: 16px;
    margin-top: 14px;
    padding: 18px 14px 14px 14px;
    color: #21331a;
    font-size: 14px;
    font-weight: 750;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 15px;
    padding: 0 7px;
    background: #ffffff;
}

QLineEdit,
QSpinBox,
QComboBox {
    min-height: 38px;
    padding: 0 11px;
    color: #263a1e;
    background: #f8faf3;
    border: 1px solid #d3e0c0;
    border-radius: 9px;
}

QLineEdit:focus,
QSpinBox:focus,
QComboBox:focus {
    border: 1px solid #7cb342;
    background: #ffffff;
}

QLineEdit:disabled,
QSpinBox:disabled,
QComboBox:disabled {
    color: #a3ad98;
    background: #eef2e8;
}

QPushButton {
    min-height: 40px;
    padding: 0 15px;
    color: #2d4023;
    background: #eff5e6;
    border: 1px solid #d6e2c4;
    border-radius: 10px;
    font-weight: 700;
}

QPushButton:hover {
    background: #e6f0d8;
    border-color: #b2c79a;
}

QPushButton:pressed,
QPushButton:checked {
    background: #cfe6ab;
    border-color: #7cb342;
}

QPushButton:disabled {
    color: #a7b19c;
    background: #f2f5ee;
    border-color: #e4ead9;
}

QPushButton[role="primary"] {
    color: #ffffff;
    background: #689f38;
    border-color: #689f38;
}

QPushButton[role="primary"]:hover {
    background: #558b2f;
}

QPushButton[role="primary"]:checked {
    color: #ffffff;
    background: #33691e;
    border-color: #33691e;
}

QPushButton[role="drive"] {
    min-width: 82px;
    min-height: 64px;
    color: #33472a;
    background: #f2f7e9;
    border-color: #cddbb5;
    font-size: 13px;
}

QPushButton[role="drive"]:pressed {
    color: #ffffff;
    background: #689f38;
    border-color: #689f38;
}

QPushButton[role="stop"] {
    min-width: 82px;
    min-height: 64px;
    color: #a12e3a;
    background: #fff0f1;
    border-color: #efc2c7;
}

QPushButton#EmergencyButton {
    min-height: 62px;
    color: #ffffff;
    background: #d93648;
    border: 1px solid #d93648;
    border-radius: 13px;
    font-size: 15px;
    font-weight: 850;
}

QPushButton#EmergencyButton:hover {
    background: #c42c3e;
}

QPushButton#EmergencyButton:pressed {
    background: #9d2030;
}

QScrollArea,
QWidget#SidePanel {
    background: transparent;
    border: 0;
}

QScrollBar:vertical {
    width: 8px;
    background: transparent;
}

QScrollBar::handle:vertical {
    min-height: 28px;
    background: #b7c99f;
    border-radius: 4px;
}

QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical,
QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical {
    height: 0;
    background: transparent;
}
"""
