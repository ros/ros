cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

add_java_source_dir(${PROJECT_SOURCE_DIR}/test)

rospack_add_java_executable(bin/Testee Testee)
rospack_add_java_executable(bin/Talker Talker)
rospack_add_java_executable(bin/Listener Listener)
rospack_add_java_executable(bin/AddTwoIntsServer AddTwoIntsServer)
rospack_add_java_executable(bin/AddTwoIntsClient AddTwoIntsClient)

rosbuild_add_executable(bin/tester test/tester.cpp)
rosbuild_add_gtest_build_flags(bin/tester)
rosbuild_add_rostest_future(test/rosjava_test.launch)


