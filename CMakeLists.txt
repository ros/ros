cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CMAKE_SKIP_BUILD_RPATH FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
SET(CMAKE_INSTALL_RPATH "${PROJECT_SOURCE_DIR}/lib")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

rosbuild_add_library(chunked_file src/chunked_file.cpp)
target_link_libraries(chunked_file bz2)

rosbuild_add_library(bag src/bag.cpp src/buffer.cpp src/message_instance.cpp)
target_link_libraries(bag chunked_file)

rosbuild_add_executable(recorder src/recorder.cpp)
rosbuild_link_boost(recorder iostreams system)
target_link_libraries(recorder chunked_file bag)

#rosbuild_add_executable(player src/player.cpp src/time_publisher.cpp src/player.cpp src/multi_player.cpp)
#rosbuild_link_boost(player iostreams system)

INSTALL(TARGETS recorder #player
        RUNTIME DESTINATION $ENV{ROS_ROOT}/bin)

#rosbuild_add_pyunit(test/test_rosrecord_offline.py)
rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest chunked_file bag)
