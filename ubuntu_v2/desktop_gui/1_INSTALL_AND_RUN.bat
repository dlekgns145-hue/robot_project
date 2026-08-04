@echo off
setlocal
cd /d "%~dp0"

echo ================================================
echo   Robot Control v2 - Windows install and run
echo ================================================
echo.
echo This may take several minutes on the first run.
echo Do not close this window while packages install.
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup_gui_windows.ps1"
if errorlevel 1 (
    echo.
    echo Installation failed.
    echo Install Python 3.10 or newer from python.org and try again.
    pause
    exit /b 1
)

echo.
echo Installation complete. Starting Robot Control v2...
start "" wscript.exe "%~dp0run_gui_windows.vbs"
exit /b 0
