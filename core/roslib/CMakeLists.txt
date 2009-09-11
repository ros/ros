cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(roslib)
genmsg()
rospack_add_pyunit(test/test_roslib_rosenv.py)
rospack_add_pyunit(test/test_roslib_message.py)
rospack_add_pyunit(test/test_roslib_msgs.py)
rospack_add_pyunit(test/test_roslib_names.py)
rospack_add_pyunit(test/test_roslib_network.py)
rospack_add_pyunit(test/test_roslib_params.py)
rospack_add_pyunit(test/test_roslib_rostime.py)
rospack_add_pyunit(test/test_roslib_genpy.py)
rospack_add_pyunit(test/test_roslib_substitution_args.py)

# To get system info used in time library
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rospack_add_boost_directories()

rospack_add_library(rostime src/time.cpp)
rospack_link_boost(rostime thread)

rospack_add_gtest(test_time test/time.cpp)
target_link_libraries(test_time rostime)
