@echo off
title --- ROS2 GAZEBO SIMULATION (HOUSE WORLD) ---
color 0E
cls
echo ========================================================
echo       KHOI DONG MO PHONG TRONG CAN NHA (FINAL)
echo ========================================================
echo.
echo DANG KHOI DONG... 
echo (Vui long doi khoang 10-20 giay de nạp canh vat 3D)
echo.
echo ========================================================
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && cd /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws && colcon build --packages-select my_robot_description my_robot_bringup && source install/setup.bash && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/gazebo_plugins/models && ros2 launch my_robot_bringup my_robot_gazebo.launch.py world:=/mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/src/my_robot_bringup/worlds/final_project.world"
pause
