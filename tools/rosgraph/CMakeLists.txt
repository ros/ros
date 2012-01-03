cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin)

catkin_project(rosgraph
  PYTHONPATH src
  )

install(FILES scripts/rosgraph
  DESTINATION share/rosgraph/bin
  )

enable_python(rosgraph)
