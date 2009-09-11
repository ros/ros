cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()
genmsg()
gensrv()
add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL) 
