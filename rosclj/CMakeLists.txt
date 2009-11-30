cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

add_clj_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_clj_repl(bin/clj)

rosbuild_gensrv()
rosbuild_genmsg()
