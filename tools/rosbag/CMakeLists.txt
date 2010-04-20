cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
SET(CMAKE_INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

rosbuild_add_library(rosbag 
	src/bag.cpp src/buffer.cpp src/bz2_stream.cpp src/chunked_file.cpp src/message_info.cpp src/player.cpp 
	src/query.cpp src/recorder.cpp src/stream.cpp src/uncompressed_stream.cpp src/view.cpp) # src/zlib_stream.cpp)
target_link_libraries(rosbag bz2 z)

rosbuild_add_executable(record src/record.cpp)
target_link_libraries(record rosbag)

rosbuild_add_executable(play src/play.cpp)
target_link_libraries(play rosbag)

INSTALL(TARGETS record play
        RUNTIME DESTINATION $ENV{ROS_ROOT}/bin)

rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest rosbag)
