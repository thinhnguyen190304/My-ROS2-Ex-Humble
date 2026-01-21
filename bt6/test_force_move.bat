@echo off
echo Dang gui lenh di chuyen thang trong 3 giay...
echo Neu robot di chuyen, nghia la loi do ban phim khong nhan tin hieu.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\""
pause
