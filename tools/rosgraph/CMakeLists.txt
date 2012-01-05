cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin)

install(FILES manifest.xml
  DESTINATION share/rosgraph
  )
install(PROGRAMS scripts/rosgraph
  DESTINATION share/rosgraph/scripts
  )
