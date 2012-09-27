cmake_minimum_required(VERSION 2.8)
project(rosbuild)
find_package(catkin REQUIRED)
catkin_package()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
install(DIRECTORY ./
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild
  PATTERN "*.cmake"
  PATTERN "*.xml"
  PATTERN "tests/*.py")
install(PROGRAMS
  bin/check_same_directories.py
  bin/download_checkmd5.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/bin)
