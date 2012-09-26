cmake_minimum_required(VERSION 2.8)
project(rosbuild)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild
  PATTERN ".svn" EXCLUDE
  PATTERN "bin" EXCLUDE
  )
install(PROGRAMS
  bin/check_same_directories.py
  bin/download_checkmd5.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/bin)
