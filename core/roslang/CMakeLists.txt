cmake_minimum_required(VERSION 2.8.3)
project(roslang)
find_package(catkin REQUIRED)
catkin_package()

install(FILES cmake/roslang.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
