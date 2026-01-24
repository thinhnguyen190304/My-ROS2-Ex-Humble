@echo off
title --- GRASP CONTROL ---
color 0B
cls
echo ========================================================
echo           LOGIC GRASP CONTROL (Attach/Detach)
echo ========================================================
echo.
echo 1. Bam phim [A] de GAP (Attach)
echo 2. Bam phim [D] de NHA (Detach)
echo 3. Bam [Ctrl+C] de THOAT
echo.
echo ========================================================
echo.

:menu
set /p userinput="Lua chon (a/d): "

if /i "%userinput%"=="a" (
    echo Dang GAP...
    wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && python3 /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/scripts/grasp_logic.py attach"
    goto menu
)

if /i "%userinput%"=="d" (
    echo Dang NHA...
    wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && python3 /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/scripts/grasp_logic.py detach"
    goto menu
)

echo Lua chon khong hop le!
goto menu
