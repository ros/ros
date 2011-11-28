cmake_minimum_required(VERSION 2.8)
project(roslib)
find_package(catkin)

install_cmake_infrastructure(roslib
  VERSION 0.0.1
  PYTHONPATH src
  )

catkin_package(roslib)
enable_python(roslib)


