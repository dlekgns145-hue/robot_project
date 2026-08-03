@echo off
cd /d %~dp0

echo Robot connection test.
echo Requirements: robot ON + Wi-Fi connected to robot network + bridge running on robot
echo WARNING: lift the wheels off the ground before continuing!
pause

call conda activate robot_vision
python robot_sender.py

echo.
echo If wheels spun 2 seconds and stopped: SUCCESS.
pause
