@echo off
echo DANG DON DEP DU AN DE GUI DI...
echo ---------------------------------------------------
echo Dang xoa thu muc RAC (build, install, log)...
echo Nguoi nhan se tu tao lai chung khi chay setup.
echo ---------------------------------------------------

cd ros2_ws
if exist build (
    rmdir /s /q build
    echo Da xoa build.
)
if exist install (
    rmdir /s /q install
    echo Da xoa install.
)
if exist log (
    rmdir /s /q log
    echo Da xoa log.
)

echo.
echo ===================================================
echo XONG! GIO BAN CO THE NEN THU MUC bt6 LAI DUOC ROI.
echo File nen se nhe hon va khong bi loi nua.
echo ===================================================
pause
