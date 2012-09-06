cmake_minimum_required(VERSION 2.8)
project(xmlrpcpp)
find_package(catkin REQUIRED COMPONENTS cpp_common)
catkin_project(xmlrpcpp
  INCLUDE_DIRS include
  LIBRARIES xmlrpcpp
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

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
install(DIRECTORY include/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS xmlrpcpp
  ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})
