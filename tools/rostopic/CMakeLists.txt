cmake_minimum_required(VERSION 2.8)
project(rostopic)
find_package(catkin)

install(FILES scripts/rostopic
  DESTINATION share/rostopic/bin
  )
