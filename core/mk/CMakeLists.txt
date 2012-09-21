cmake_minimum_required(VERSION 2.8)
project(mk)
find_package(catkin REQUIRED)
catkin_package()
catkin_package_export()

install(DIRECTORY ,
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/mk
  PATTERN ".svn" EXCLUDE)
