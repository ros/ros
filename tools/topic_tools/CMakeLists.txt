cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()

rospack_add_library(topic_tools src/shape_shifter.cpp)

rospack_add_executable(bin/switch_mux src/switch_mux.cpp)

rospack_add_executable(bin/mux src/mux.cpp)
target_link_libraries(bin/mux topic_tools)

rospack_add_executable(bin/relay src/relay.cpp)
target_link_libraries(bin/relay topic_tools)

rospack_add_executable(bin/drop src/drop.cpp)
target_link_libraries(bin/drop topic_tools)

#rospack_add_executable(demux demux.cpp)
#target_link_libraries(demux topic_tools)

rospack_add_executable(bin/throttle src/throttle.cpp)
target_link_libraries(bin/throttle topic_tools)
