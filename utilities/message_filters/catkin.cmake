cmake_minimum_required(VERSION 2.8)
project(message_filters)
find_package(Boost COMPONENTS signals)

include_directories(include)
include_directories(include/message_filters) # <-weird
include_directories(${ROS_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(message_filters SHARED src/connection.cpp)
target_link_libraries(message_filters ${Boost_LIBRARIES})

install_cmake_infrastructure(message_filters
  VERSION 0.0.0
  INCLUDE_DIRS include
  LIBRARIES message_filters
  )


