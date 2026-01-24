@echo off
title --- PRO ARM CONTROL (GAZEBO) ---
color 0E
cls
echo ========================================================
echo        DIEU KHIEN CANH TAY THAT TRONG GAZEBO
echo ========================================================
echo.
echo HUONG DAN:
echo 1. Một cua sổ RQT sẽ hiện ra.
echo 2. Chọn "controller_manager" trong danh sách (nếu chua chọn).
echo 3. Chọn "arm_controller" để điều khiển 2 khớp tay.
echo 4. Nhấn nút [ON] màu đỏ để kích hoạt thanh trượt.
echo.
echo => LƯU Ý: CÁI NÀY ĐIỀU KHIỂN ROBOT THẬT TRONG GAZEBO!
echo ========================================================
echo.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller"
pause
