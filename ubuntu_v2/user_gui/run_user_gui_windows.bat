@echo off
setlocal
set "USER_GUI_DIR=%~dp0"
set "USER_GUI_PYTHON=%USER_GUI_DIR%.venv\Scripts\python.exe"
set "USER_GUI_LOG=%USER_GUI_DIR%user_gui_error.log"

if not exist "%USER_GUI_PYTHON%" (
    echo Robot Companion is not installed.
    echo Run 1_INSTALL_AND_RUN_WINDOWS.bat first.
    pause
    exit /b 1
)

echo Robot Companion starting...
echo Error log: %USER_GUI_LOG%
echo.

"%USER_GUI_PYTHON%" "%USER_GUI_DIR%main.py" 2> "%USER_GUI_LOG%"
if errorlevel 1 (
    echo.
    echo Robot Companion exited with an error.
    echo Send user_gui_error.log to the administrator.
    echo.
    type "%USER_GUI_LOG%"
    pause
    exit /b 1
)

exit /b 0
