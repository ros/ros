cmake_minimum_required(VERSION 3.0.2)
project(roslang)
find_package(catkin REQUIRED)
catkin_package()

install(FILES cmake/roslang.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
