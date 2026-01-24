@echo off
title --- SUPER ARM CONTROL (KEYBOARD) ---
color 0A
cls
echo ========================================================
echo       BANG DIEU KHIEN CANH TAY TU DO (KEYBOARD)
echo ========================================================
echo.
echo DANG KHOI DONG... 
echo (Luu ý: Phải bấm chuột vào cửa sổ này thì mới điều khiển được nhé!)
echo.
echo ========================================================
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && python3 /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/scripts/arm_teleop.py"
pause
