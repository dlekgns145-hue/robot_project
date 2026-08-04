@echo off
setlocal
set "GUI_DIR=%~dp0"
set "GUI_PYTHON=%GUI_DIR%.venv\Scripts\python.exe"

if not exist "%GUI_PYTHON%" (
    echo GUI environment is not installed.
    echo Run setup_gui_windows.ps1 first.
    pause
    exit /b 1
)

"%GUI_PYTHON%" "%GUI_DIR%main.py"
if errorlevel 1 pause
