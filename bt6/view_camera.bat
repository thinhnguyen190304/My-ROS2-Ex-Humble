@echo off
title CAMERA VIEW (RQT)
echo ===================================================
echo   DANG MO MAN HINH CAMERA...
echo ===================================================
echo.
echo HUONG DAN:
echo 1. Mot cua so phan mem se hien len.
echo 2. O goc tren ben trai, cho o chon "Topic".
echo 3. Bam vao do va chon dong: "/my_robot/camera1/image_raw"
echo.
echo (Neu chua thay topic, bam nut Refresh hinh mui ten tron xanh)
echo.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && ros2 run rqt_image_view rqt_image_view"
pause
