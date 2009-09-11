cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(rosdeb)

configure_file(bin/rosdeb $ENV{ROS_ROOT}/bin/rosdeb COPYONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES $ENV{ROS_ROOT}/bin/rosdeb)
