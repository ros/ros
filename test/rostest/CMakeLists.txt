cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_add_gtest(test/test_permuter test/test_permuter.cpp)
rosbuild_add_rostest(test/hztest0.test)
rosbuild_add_rostest(test/hztest.test)
