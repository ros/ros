cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_add_executable(pointcloud_serdes pointcloud_serdes.cpp)
#rosbuild_add_compile_flags(pointcloud_serdes "-O3 -funroll-loops")
#rosbuild_add_compile_flags(pointcloud_serdes "-march=prescott")

