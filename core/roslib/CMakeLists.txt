cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_library(roslib src/package.cpp)
rosbuild_link_boost(roslib thread)

if (NOT APPLE)
target_link_libraries(roslib rt)
endif (NOT APPLE)
