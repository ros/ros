cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#rospack_add_executable(demo src/demo/demo.cpp)

rospack_add_library(rosrecorder src/rosrecord/recorder.cpp)

rospack_add_executable(rosrecord src/rosrecord/rosrecord.cpp)
target_link_libraries(rosrecord rosrecorder)

rospack_add_executable(rosplay src/rosplay/rosplay.cpp src/rosplay/time_publisher.cpp)

INSTALL(TARGETS rosrecord rosplay
        RUNTIME DESTINATION $ENV{ROS_ROOT}/bin)

