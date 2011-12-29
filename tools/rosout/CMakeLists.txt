cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_catkinize()

rosbuild_init()
rosbuild_add_executable(rosout rosout.cpp)
