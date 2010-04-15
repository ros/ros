cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

include_directories(include/message_filters)

rosbuild_add_boost_directories()
rosbuild_add_library(message_filters src/connection.cpp)
rosbuild_link_boost(message_filters thread signals)

#rosbuild_add_executable(filter_example src/filter_example.cpp)
#target_link_libraries(filter_example message_filters)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(test EXCLUDE_FROM_ALL)
