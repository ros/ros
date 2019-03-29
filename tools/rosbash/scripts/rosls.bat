@echo off

if "%1" equ "--help" goto :usage
for /f "tokens=*" %%g in ('call python %~dp0\rosfindpath.py %1 forceeval') do (
    set target_path=%%~g
)

if /i "%target_path:~0,7%"=="Error: " (
    echo rosls: %target_path:~7%
    exit /b 1
)

dir "%target_path%"
exit /b 0

:usage
echo usage: rosls [package]
echo.
echo.
echo Lists contents of a package directory.
exit /b 0
