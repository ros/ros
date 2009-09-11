cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# Copy nodes/master to $ROS_ROOT/bin/master and zenmaster
configure_file(nodes/master $ENV{ROS_ROOT}/bin/zenmaster COPYONLY)
SET(clean_files $ENV{ROS_ROOT}/bin/zenmaster)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${clean_files}")

