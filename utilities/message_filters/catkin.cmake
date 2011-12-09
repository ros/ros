cmake_minimum_required(VERSION 2.8)
project(message_filters)

include_directories(include)
include_directories(include/message_filters) # <-weird
include_directories(${ROS_INCLUDE_DIRS})

add_library(message_filters src/connection.cpp)

install_cmake_infrastructure(message_filters
  VERSION 0.0.0
  INCLUDE_DIRS include
  LIBRARIES message_filters
  )


