"""Friendly visual theme for the end-user robot controller."""

USER_STYLESHEET = r"""
QWidget#AppRoot {
    background: #f3f7fb;
    color: #152033;
}

QWidget {
    color: #20304a;
    font-family: "Pretendard", "Apple SD Gothic Neo", "Segoe UI", sans-serif;
    font-size: 14px;
}

QFrame#Header,
QFrame#CameraCard,
QFrame#StatusCard {
    background: #ffffff;
    border: 1px solid #dbe5ef;
    border-radius: 18px;
}

QLabel#Eyebrow {
    color: #2777df;
    font-size: 11px;
    font-weight: 800;
    letter-spacing: 1px;
}

QLabel#Title {
    color: #14213a;
    font-size: 27px;
    font-weight: 800;
}

QLabel#Subtitle,
QLabel#Hint,
QLabel#StatusCaption {
    color: #718096;
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
    color: #127159;
    background: #eafaf5;
    border: 1px solid #bde9dc;
}

QLabel#VideoSurface {
    color: #8795a8;
    background: #0c1420;
    border: 1px solid #263448;
    border-radius: 14px;
    font-size: 15px;
}

QLabel#CameraMetrics {
    color: #52657a;
    background: #edf4fb;
    border-radius: 10px;
    padding: 5px 10px;
    font-size: 11px;
}

QLabel#StatusValue {
    color: #16253e;
    font-size: 15px;
    font-weight: 750;
}

QGroupBox {
    background: #ffffff;
    border: 1px solid #dbe5ef;
    border-radius: 16px;
    margin-top: 14px;
    padding: 18px 14px 14px 14px;
    color: #192942;
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
    color: #20304a;
    background: #f8fafc;
    border: 1px solid #cfdbe7;
    border-radius: 9px;
}

QLineEdit:focus,
QSpinBox:focus,
QComboBox:focus {
    border: 1px solid #3688ef;
    background: #ffffff;
}

QLineEdit:disabled,
QSpinBox:disabled,
QComboBox:disabled {
    color: #9aa8b8;
    background: #eef2f6;
}

QPushButton {
    min-height: 40px;
    padding: 0 15px;
    color: #243650;
    background: #edf3f9;
    border: 1px solid #d1dce7;
    border-radius: 10px;
    font-weight: 700;
}

QPushButton:hover {
    background: #e2edf8;
    border-color: #aebfd0;
}

QPushButton:pressed,
QPushButton:checked {
    background: #cfe4fa;
    border-color: #5b9de5;
}

QPushButton:disabled {
    color: #a3afbd;
    background: #f1f4f7;
    border-color: #e2e8ef;
}

QPushButton[role="primary"] {
    color: #ffffff;
    background: #2678df;
    border-color: #2678df;
}

QPushButton[role="primary"]:hover {
    background: #1d69c7;
}

QPushButton[role="primary"]:checked {
    color: #ffffff;
    background: #1559ad;
    border-color: #1559ad;
}

QPushButton[role="drive"] {
    min-width: 82px;
    min-height: 64px;
    color: #27405d;
    background: #f1f6fb;
    border-color: #cbd9e7;
    font-size: 13px;
}

QPushButton[role="drive"]:pressed {
    color: #ffffff;
    background: #2777df;
    border-color: #2777df;
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
    background: #bcc9d6;
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
