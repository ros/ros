if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()

rosbuild_add_pyunit(test/test_rostopic_command_line_offline.py)
rosbuild_add_pyunit(test/test_rostopic_unit.py)
rosbuild_add_rostest(test/rostopic.test)
