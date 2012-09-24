cmake_minimum_required(VERSION 2.8)
project(mk)
find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY .
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/mk
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE
  PATTERN ".svn" EXCLUDE)
