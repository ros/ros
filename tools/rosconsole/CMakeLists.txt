project(rosconsole)

# See ticket: https://code.ros.org/trac/ros/ticket/3626
# On mac use g++-4.2
IF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")
  IF(EXISTS "/usr/bin/g++-4.2")
    set(CMAKE_CXX_COMPILER /usr/bin/g++-4.2)
  ELSE(EXISTS "/usr/bin/g++-4.2")
    # If there is no g++-4.2 use clang++
    set(CMAKE_CXX_COMPILER /usr/bin/clang++)
  ENDIF(EXISTS "/usr/bin/g++-4.2")
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-11.*")

find_package(ROS COMPONENTS catkin cpp_common rostime)

include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost COMPONENTS regex thread)

include_directories(include)

add_library(rosconsole SHARED src/rosconsole/rosconsole.cpp)

find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()
target_link_libraries(rosconsole ${ROS_LIBRARIES} ${LOG4CXX_LIBRARY} ${Boost_LIBRARIES})

install(TARGETS rosconsole
    DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

catkin_project(rosconsole
  LIBRARIES rosconsole
  INCLUDE_DIRS include
  )
