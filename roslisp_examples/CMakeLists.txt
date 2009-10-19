cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

gensrv()
genmsg()

#rospack_add_lisp_executable(bin/add-two-ints-client src/add-two-ints-client.lisp)
#rospack_add_lisp_executable(bin/listener src/listener.lisp)
#rospack_add_lisp_executable(bin/add-two-ints-server src/add-two-ints-server.lisp)
#rospack_add_lisp_executable(bin/talker src/talker.lisp)
#rospack_add_lisp_executable(bin/params src/params.lisp)
#rospack_add_lisp_executable(bin/rosout-example src/rosout-example.lisp)
#
#
#

rospack_add_lisp_executable(bin/talker roslisp_examples roslisp-examples:talker)
rospack_add_lisp_executable(bin/listener roslisp_examples roslisp-examples:listener)
rospack_add_lisp_executable(bin/add-two-ints-client roslisp_examples roslisp-examples:add-two-ints-client)
rospack_add_lisp_executable(bin/add-two-ints-server roslisp_examples roslisp-examples:add-two-ints-server)
rospack_add_lisp_executable(bin/params-example roslisp_examples roslisp-examples:params-example)
rospack_add_lisp_executable(bin/rosout-example roslisp_examples roslisp-examples:rosout-example)
rospack_add_lisp_executable(bin/array-talker roslisp_examples roslisp-examples:array-talker)
rospack_add_lisp_executable(bin/array-listener roslisp_examples roslisp-examples:array-listener)



## tests
#rospack_add_lisp_executable(bin/listener-test roslisp-examples-test roslisp-listener-test:main)
# rospack_add_lisp_executable(bin/talker-test roslisp-examples-test roslisp-talker-test:main)
# rospack_add_executable(bin/cpp-tester test/tester.cpp)
# rospack_add_gtest_build_flags(bin/cpp-tester)
# rospack_add_rostest(test/talker_listener_test.launch)
