@echo off
setlocal
cd /d "%~dp0"

echo ==================================================
echo   Robot Companion - Windows install and run
echo ==================================================
echo.
echo The first installation may take several minutes.
echo Keep this window open while packages are installed.
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_user_gui_windows.ps1"
if errorlevel 1 (
    echo.
    echo Installation failed.
    echo Install Python 3.10 or newer from python.org and try again.
    pause
    exit /b 1
)

echo.
echo Installation complete. Starting Robot Companion...
call "%~dp0run_user_gui_windows.bat"
exit /b %errorlevel%
