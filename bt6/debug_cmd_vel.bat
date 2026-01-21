@echo off
echo Dang ghi log topic /cmd_vel...
echo Vui long chay qua trinh dieu khien robot, bam vai phim di chuyen.
echo Sau khoang 10 giay, quay lai cua so nay va bam Ctrl+C.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /cmd_vel"
pause
