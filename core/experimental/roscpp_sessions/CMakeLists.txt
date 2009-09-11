cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(roscpp_sessions)

gensrv()

add_subdirectory(test EXCLUDE_FROM_ALL)
