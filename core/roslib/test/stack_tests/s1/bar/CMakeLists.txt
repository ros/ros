cmake_minimum_required(VERSION 3.0.2)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROSPACK_MAKEDIST true)
rosbuild_make_distribution(1.5.0-cmake)
