cmake_minimum_required(VERSION 2.8)
project(message_filters)
find_package(Boost COMPONENTS signals)

include_directories(include)
include_directories(${ROS_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(message_filters SHARED src/connection.cpp)
target_link_libraries(message_filters ${Boost_LIBRARIES})

install(TARGETS message_filters
    DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

catkin_project(message_filters
  INCLUDE_DIRS include
  LIBRARIES message_filters
  )

