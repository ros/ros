cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin)

install(FILES scripts/rosgraph
  DESTINATION share/rosgraph/bin
  )
