cmake_minimum_required(VERSION 2.8)
project(XmlRpc)

find_package(ROS COMPONENTS catkin cpp_common)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(src)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(XmlRpc SHARED
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  )

if(WIN32)
  target_link_libraries(XmlRpc ws2_32)
endif()

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

#explicitly install library and includes
install(TARGETS XmlRpc
  DESTINATION lib
 )

catkin_project(XmlRpc
  INCLUDE_DIRS src
  LIBRARIES XmlRpc
  )
