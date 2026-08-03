@echo off
cd /d %~dp0

echo YOLO Follow-Me - robot run
echo Requirements before continuing:
echo   1. Robot ON, laptop Wi-Fi connected to the robot's network
echo   2. robot_cmd_bridge.py running on robot ("cmd_bridge ready. listening on 9999")
echo   3. base_node_X3 (or equivalent) running on robot
echo   4. yolo_follow_robot.py -> ROBOT_IP set to the robot's CURRENT ip address
echo   5. Robot placed on open floor, 2m+ away from walls (NOT on the test stand)
pause

call conda activate robot_vision
python yolo_follow_robot.py

echo.
echo Program ended (q was pressed, or an error occurred above).
pause
