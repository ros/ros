@echo off

if "%1" equ "--help" goto :usage
if not "%2" equ "" goto :usage

if "%1" equ "" (
    if not "%ROS_WORKSPACE%" equ "" (
        cd /d %ROS_WORKSPACE%
        exit /b 0
    )

    if not "%CMAKE_PREFIX_PATH%" equ "" (
        for %%a in ("%CMAKE_PREFIX_PATH:;=";"%") do (
            if exist %%a\.catkin (
                cd /d %%a
                exit /b 0
            )
        )
    )

    echo Neither ROS_WORKSPACE is set nor a catkin workspace is listed in CMAKE_PREFIX_PATH.  Please set ROS_WORKSPACE or source a catkin workspace to use roscd with no arguments.
    exit /b 1
)

for /f "tokens=*" %%g in ('call python %~dp0\rosfindpath.py %1 forceeval') do (
    set target_path=%%~g
)

if /i "%target_path:~0,7%"=="Error: " (
    echo roscd: %target_path:~7%
    exit /b 1
)

if "%target_path%" equ "" (
    if not "%ROS_WORKSPACE%" equ "" (
        cd /d %ROS_WORKSPACE%
        exit /b 0
    )

    echo No ROS_WORKSPACE set.  Please set ROS_WORKSPACE to use roscd with no arguments.
    exit /b 1
)

cd /d "%target_path%"
exit /b 0

:usage
echo usage: roscd package
echo.
echo.
echo Jump to target package.
exit /b 0
