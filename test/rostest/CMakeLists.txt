cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# Copy of rostest to top level bin plus clean target for it as
# configure_file does not auto-clean. Why is it so klunky to copy 
# files in cmake?
configure_file(bin/rostest $ENV{ROS_ROOT}/bin/rostest COPYONLY)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES $ENV{ROS_ROOT}/bin/rostest)

rospack_add_gtest(test/test_permuter test/test_permuter.cpp)
