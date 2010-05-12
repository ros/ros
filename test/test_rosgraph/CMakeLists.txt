cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_pyunit(test/test_rosgraph_command_offline.py)
rosbuild_add_pyunit(test/test_rosgraph_masterapi_offline.py)
rosbuild_add_rostest(test/masterapi.test)
