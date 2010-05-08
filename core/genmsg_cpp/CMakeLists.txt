cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_add_executable(genmsg_lisp genmsg_lisp.cpp genmsg_main_lisp.cpp utils.cpp)
rosbuild_add_executable(gensrv_lisp gensrv_lisp.cpp genmsg_lisp.cpp utils.cpp)
rosbuild_add_executable(genmsg_oct genmsg_oct.cpp genmsg_main_oct.cpp utils.cpp)
rosbuild_add_executable(gensrv_oct gensrv_oct.cpp genmsg_oct.cpp utils.cpp)
rosbuild_add_executable(genmsg_java genmsg_java.cpp genmsg_main_java.cpp utils.cpp)
rosbuild_add_executable(gensrv_java gensrv_java.cpp genmsg_java.cpp utils.cpp)

