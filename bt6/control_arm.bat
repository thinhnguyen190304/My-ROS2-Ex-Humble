@echo off
title --- MOVEIT ARM CONTROL ---
color 0D
cls
echo ========================================================
echo        DIEU KHIEN CANH TAY BANG MOVEIT (RViz)
echo ========================================================
echo.
echo HUONG DAN:
echo 1. Cho RViz hien len (co the mat 10-20 giay).
echo 2. Ben trai panel "Motion Planning", chon tab "Planning".
echo 3. Dung chuot keo cac "Interactive Markers" (vong tron mau)
echo    dau canh tay de chon vi tri moi.
echo 4. Bam nut "Plan" de tinh toan duong di.
echo 5. Bam nut "Execute" de robot thuc hien (neu simulation dang chay).
echo.
echo ========================================================
echo.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && ros2 launch my_robot_moveit_config moveit.launch.py"
pause
