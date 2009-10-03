cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()

# To get system info used in time library
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_boost_directories()

rosbuild_add_library(roslib src/time.cpp src/rate.cpp src/package.cpp)
rosbuild_link_boost(roslib thread)

rosbuild_add_gtest(test_time test/time.cpp)
target_link_libraries(test_time roslib)

rosbuild_add_gtest(test_package test/package.cpp)
target_link_libraries(test_package roslib)
 
