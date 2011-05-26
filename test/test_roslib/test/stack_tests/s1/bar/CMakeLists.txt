cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROSPACK_MAKEDIST true)
rosbuild_make_distribution(1.5.0-cmake)
