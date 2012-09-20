cmake_minimum_required(VERSION 2.8)
project(roslang)
find_package(catkin REQUIRED)
catkin_package_export()

install(FILES cmake/roslang.cmake
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/cmake)
