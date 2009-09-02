cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

genmsg()


# Tests
rospack_add_lisp_executable(bin/listener-test roslisp-test roslisp-listener-test:main)
rospack_add_lisp_executable(bin/talker-test roslisp-test roslisp-talker-test:main)
rospack_add_executable(bin/cpp-tester test/tester.cpp)
rospack_add_gtest_build_flags(bin/cpp-tester)
rospack_add_rostest_future(test/talker_listener_test.launch)


rospack_add_lisp_executable(bin/listener-test roslisp-test roslisp-listener-test:main)
rospack_add_lisp_executable(bin/talker-test roslisp-test roslisp-talker-test:main)
rospack_add_lisp_executable(bin/complex-listener-test roslisp-test roslisp-complex-listener-test:main)
rospack_add_lisp_executable(bin/complex-talker-test roslisp-test roslisp-complex-talker-test:main)
