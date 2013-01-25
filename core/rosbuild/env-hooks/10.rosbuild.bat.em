REM generated from rosbuild/env-hooks/10.rosbuild.bat.em

@[if DEVELSPACE]@
REM env variables in develspace
set ROS_ROOT=@(CMAKE_CURRENT_SOURCE_DIR)
@[else]@
REM env variables in installspace
set ROS_ROOT=@(CMAKE_INSTALL_PREFIX)/@(CATKIN_GLOBAL_SHARE_DESTINATION)/ros
@[end if]@
