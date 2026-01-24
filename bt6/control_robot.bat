@echo off
color 0B
title --- REMOTE LAI XE + GAP DO ---
cls
echo ========================================================
echo        REMOTE LAI XE & GIAO DO (Sieu Cap Vip Pro)
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
echo --------------------------------------------------------
echo         CANH TAY ROBOT (MAY GAP)
echo --------------------------------------------------------
echo    [ T ] : Vai LEN       [ Y ] : Khuyu LEN
echo    [ G ] : Vai XUONG     [ H ] : Khuyu XUONG
echo.
echo    [ R ] : MO KEP (Nha do)
echo    [ F ] : DONG KEP (Gap do)
echo.
echo    [ V ] : Reset tay ve vi tri ban dau
echo.
echo ========================================================
echo  QUAN TRONG: Click chuot vao day truoc khi bam phim!
echo.
wsl -d Ubuntu-22.04 -- bash -c "source /opt/ros/humble/setup.bash && python3 /mnt/d/Documents/ROS2_Course_Projects_Learn/bt6/simple_control.py"
