cmake_minimum_required(VERSION 2.8)
project(xmlrpcpp)
catkin_project(xmlrpcpp
  INCLUDE_DIRS src
  LIBRARIES xmlrpcpp
  )

find_package(ROS COMPONENTS catkin cpp_common)
include_directories(${ROS_INCLUDE_DIRS})

include_directories(src)

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp 
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
  target_link_libraries(xmlrpcpp ws2_32)
endif()

message("TODO: consider moving xmlrpcpp's headers into a subdirectory")
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

#explicitly install library and includes
install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
 )
