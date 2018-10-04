REM generated from rosbuild/env-hooks/10.rosbuild.bat.em

@[if DEVELSPACE]@
REM env variables in develspace
set ROS_ROOT=@(CMAKE_CURRENT_SOURCE_DIR)
@[else]@
REM env variables in installspace
if "%CATKIN_ENV_HOOK_WORKSPACE%"=="" set CATKIN_ENV_HOOK_WORKSPACE=@(CMAKE_INSTALL_PREFIX)
set ROS_ROOT=%CATKIN_ENV_HOOK_WORKSPACE%/@(CATKIN_GLOBAL_SHARE_DESTINATION)/ros
@[end if]@
