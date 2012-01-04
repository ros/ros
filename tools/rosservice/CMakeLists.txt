cmake_minimum_required(VERSION 2.8)
project(rosservice)
find_package(catkin)

install(FILES scripts/rosservice
  DESTINATION share/rosservice/bin
  )
