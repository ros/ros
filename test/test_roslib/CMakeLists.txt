if(ROSBUILD)
  # this is for rosbuild2 integration, you can ignore this
  include(rosbuild.cmake OPTIONAL)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
# unit tests
rosbuild_add_pyunit(test/test_roslib.py)
rosbuild_add_pyunit(test/test_roslib_exceptions.py)
rosbuild_add_pyunit(test/test_roslib_manifest.py)
rosbuild_add_pyunit(test/test_roslib_manifestlib.py)
rosbuild_add_pyunit(test/test_roslib_os_detect.py)
rosbuild_add_pyunit(test/test_roslib_names.py)
rosbuild_add_pyunit(test/test_roslib_network.py)
rosbuild_add_pyunit(test/test_roslib_packages.py)
rosbuild_add_pyunit(test/test_roslib_params.py)
rosbuild_add_pyunit(test/test_roslib_rosenv.py)
rosbuild_add_pyunit(test/test_roslib_rospack.py)
rosbuild_add_pyunit(test/test_roslib_rostime.py)
rosbuild_add_pyunit(test/test_roslib_scriptutil.py)
rosbuild_add_pyunit(test/test_roslib_stacks.py)
rosbuild_add_pyunit(test/test_roslib_stack_manifest.py)
rosbuild_add_pyunit(test/test_roslib_substitution_args.py)
rosbuild_add_pyunit(test/test_roslib_xmlrpc.py)
 
#integration tests

rosbuild_add_boost_directories()
rosbuild_add_gtest(test/utest test/utest.cpp)
rosbuild_link_boost(test/utest thread)


rosbuild_add_gtest(test_package test/package.cpp)
target_link_libraries(test_package roslib)
