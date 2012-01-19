cmake_minimum_required(VERSION 2.8)
project(rosunit)
find_package(catkin)
catkin_project(${PROJECT_NAME}
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
  )
