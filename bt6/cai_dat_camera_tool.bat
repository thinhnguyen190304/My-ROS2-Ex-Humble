@echo off
echo Dang cai dat cong cu xem Camera...
echo Vui long nhap mat khau Ubuntu neu duoc hoi.
wsl -d Ubuntu-22.04 -- bash -c "sudo apt update && sudo apt install ros-humble-image-view -y"
echo.
echo Cai dat xong!
pause
