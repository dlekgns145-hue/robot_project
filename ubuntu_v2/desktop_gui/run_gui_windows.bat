@echo off
setlocal
set "GUI_DIR=%~dp0"
set "GUI_PYTHON=%GUI_DIR%.venv\Scripts\python.exe"
set "GUI_LOG=%GUI_DIR%gui_error.log"

if not exist "%GUI_PYTHON%" (
    echo GUI environment is not installed.
    echo Run setup_gui_windows.ps1 first.
    pause
    exit /b 1
)

echo Robot Control v2 starting...
echo Error log: %GUI_LOG%
echo.

"%GUI_PYTHON%" "%GUI_DIR%main.py" 2> "%GUI_LOG%"
if errorlevel 1 (
    echo.
    echo GUI exited with an error.
    echo Send the contents of gui_error.log for diagnosis.
    echo.
    type "%GUI_LOG%"
    pause
    exit /b 1
)

exit /b 0
