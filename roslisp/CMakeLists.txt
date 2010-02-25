cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

# Tests
rospack_add_lisp_executable(bin/listener-test roslisp-test roslisp-test:listener)
rospack_add_lisp_executable(bin/talker-test roslisp-test roslisp-test:talker)
rospack_add_lisp_executable(bin/complex-listener-test roslisp-test roslisp-test:complex-listener)
rospack_add_lisp_executable(bin/complex-talker-test roslisp-test roslisp-test:complex-talker)
rospack_add_lisp_executable(bin/client-test roslisp-test roslisp-test:service-client)
rospack_add_lisp_executable(bin/server-test roslisp-test roslisp-test:service-server)
rosbuild_add_executable(bin/cpp-tester test/tester.cpp)
rosbuild_add_gtest_build_flags(bin/cpp-tester)
rosbuild_add_rostest(test/roslisp_test.launch)


