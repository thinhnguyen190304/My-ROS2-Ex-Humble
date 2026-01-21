@echo off
color 0B
title --- REMOTE LAI XE PRO ---
cls
echo ========================================================
echo        REMOTE LAI XE PRO  (Version 2.0)
echo ========================================================
echo.
echo    [ U ]      [ I ]      [ O ]
echo   (Cua Trai) (Di Thang) (Cua Phai)
echo        \      ^|      /
echo         \     ^|     /
echo  [ J ] ---( VO LANG )--- [ L ]
echo (Xoay Trai)         (Xoay Phai)
echo               ^|
echo               ^|
echo             [ , ]
echo            (Di Lui)
echo.
echo      [ K ] : PHANH CHAN (Dung lai!)
echo.
echo ========================================================
echo  CHON TOC DO:
echo    [ 1 ] : Cham
echo    [ 2 ] : Vua  (Di mac dinh)
echo    [ 3 ] : SIUE NHANH (Bao dom)
echo ========================================================
echo.
echo  LUU Y: May tinh khong nhan duoc 2 phim cung luc.
echo         De vua di vua re, hay bam phim U hoac O !
echo.
echo  Click chuot vao day truoc khi bam phim nhe!
echo.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && python3 /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/simple_control.py"
