@echo off
echo Dang khoi dong mo phong ROS 2 trong WSL...
echo Nhan Ctrl+C de dung lai.
wsl -d Ubuntu-22.04 -- bash -c "export GAZEBO_MODEL_DATABASE_URI='' && export LIBGL_ALWAYS_SOFTWARE=1 && source /opt/ros/humble/setup.bash && source /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/ros2_ws/install/setup.bash && ros2 launch my_robot_bringup my_robot_gazebo.launch.xml"
pause
