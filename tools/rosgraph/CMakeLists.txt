cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin)

install_cmake_infrastructure(rosgraph
  VERSION 0.0.1
  PYTHONPATH src
  )

enable_python(rosgraph)
