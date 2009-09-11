cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(roslisp_examples)

gensrv()
rospack_add_lisp_executable(bin/add-two-ints-client src/add-two-ints-client.lisp)
rospack_add_lisp_executable(bin/listener src/listener.lisp)
rospack_add_lisp_executable(bin/add-two-ints-server src/add-two-ints-server.lisp)
rospack_add_lisp_executable(bin/talker src/talker.lisp)
rospack_add_lisp_executable(bin/params src/params.lisp)
rospack_add_lisp_executable(bin/rosout-example src/rosout-example.lisp)

# Temporarily commented out due to error about wild pathname
#rospack_add_lisp_executable(bin/array-talker src/array-talker.lisp)
#rospack_add_lisp_executable(bin/array-listener src/array-listener.lisp)

