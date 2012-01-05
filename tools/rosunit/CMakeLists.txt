cmake_minimum_required(VERSION 2.8)
project(rosunit)
find_package(catkin)

install(DIRECTORY scripts
  DESTINATION share/rosunit/bin
  )
