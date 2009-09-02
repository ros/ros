cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

genmsg()
gensrv()

add_java_source_dir(${PROJECT_SOURCE_DIR}/test)

rospack_add_java_executable(bin/Testee Testee)

rospack_add_executable(bin/tester test/tester.cpp)
rospack_add_gtest_build_flags(bin/tester)
rospack_add_rostest_future(test/rosjava_test.launch)


