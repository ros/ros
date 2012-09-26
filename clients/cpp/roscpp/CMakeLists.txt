cmake_minimum_required(VERSION 2.8)
project(roscpp)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  cpp_common rostime rosconsole roscpp_serialization
  roscpp_traits std_msgs rosgraph_msgs xmlrpcpp rosunit
  rostest
  )

catkin_package(
  LIBRARIES roscpp
  INCLUDE_DIRS include
  DEPENDS cpp_common rostime rosconsole roscpp_serialization
          roscpp_traits std_msgs rosgraph_msgs xmlrpcpp
  )

include_directories(${catkin_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

include_directories(include)

include(CheckIncludeFiles)
include(CheckFunctionExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)

# Output test results to config.h
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/src/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(roscpp 
  src/libros/master.cpp
  src/libros/network.cpp
  src/libros/subscriber.cpp
  src/libros/common.cpp
  src/libros/publisher_link.cpp
  src/libros/service_publication.cpp
  src/libros/connection.cpp
  src/libros/single_subscriber_publisher.cpp
  src/libros/param.cpp
  src/libros/service_server.cpp
  src/libros/wall_timer.cpp
  src/libros/xmlrpc_manager.cpp
  src/libros/publisher.cpp
  src/libros/timer.cpp
  src/libros/io.cpp
  src/libros/names.cpp
  src/libros/topic.cpp
  src/libros/topic_manager.cpp
  src/libros/poll_manager.cpp
  src/libros/publication.cpp
  src/libros/intraprocess_subscriber_link.cpp
  src/libros/intraprocess_publisher_link.cpp
  src/libros/callback_queue.cpp
  src/libros/service_server_link.cpp
  src/libros/service_client.cpp
  src/libros/node_handle.cpp
  src/libros/connection_manager.cpp
  src/libros/file_log.cpp
  src/libros/transport/transport_udp.cpp
  src/libros/transport/transport_tcp.cpp
  src/libros/subscriber_link.cpp
  src/libros/service_client_link.cpp
  src/libros/transport_publisher_link.cpp
  src/libros/transport_subscriber_link.cpp
  src/libros/service_manager.cpp
  src/libros/rosout_appender.cpp
  src/libros/init.cpp
  src/libros/subscription.cpp
  src/libros/subscription_queue.cpp
  src/libros/spinner.cpp
  src/libros/internal_timer_manager.cpp
  src/libros/message_deserializer.cpp
  src/libros/header.cpp
  src/libros/poll_set.cpp
  src/libros/service.cpp
  src/libros/this_node.cpp
  )

add_dependencies(roscpp roscpp_gencpp rosgraph_msgs_gencpp std_msgs_gencpp)

target_link_libraries(roscpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_message_files(
  DIRECTORY msg
  FILES Logger.msg
)

add_service_files(
  DIRECTORY srv
  FILES Empty.srv GetLoggers.srv SetLoggerLevel.srv
)

# Testing message and service files.  See comment below about subdirectory support.
add_message_files(
  NOINSTALL
  DIRECTORY test/msg
  FILES
  TestArray.msg
  TestEmpty.msg
  TestStringInt.msg
  TestWithHeader.msg
)
add_service_files(
  NOINSTALL
  DIRECTORY test/srv
  FILES
  BadTestStringString.srv
  TestStringString.srv
)

# Serialization testing message files.
add_message_files(
  NOINSTALL
  DIRECTORY test_serialization/msg
  FILES
  ArrayOfFixedLength.msg
  ArrayOfVariableLength.msg
  Constants.msg
  CustomHeader.msg
  EmbeddedExternal.msg
  EmbeddedFixedLength.msg
  EmbeddedVariableLength.msg
  FixedLengthArrayOfExternal.msg
  FixedLength.msg
  FixedLengthStringArray.msg
  HeaderNotFirstMember.msg
  VariableLengthArrayOfExternal.msg
  VariableLength.msg
  VariableLengthStringArray.msg
  WithDuration.msg
  WithHeader.msg
  WithMemberNamedHeaderThatIsNotAHeader.msg
  WithTime.msg
)

# Performance testing message files.  add_message_files() does not
# currently support being called from a subdirectory, so this is here
# for now.
add_message_files(
  NOINSTALL
  DIRECTORY perf/msg
  FILES
  LatencyMessage.msg
  ThroughputMessage.msg
)

# dependencies std_msgs and rosgraph_msgs are needed by tests, but not by roscpp otherwise.
generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)
# generate_messages()

#explicitly install library and includes
install(TARGETS roscpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/roscpp.cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild)
install(PROGRAMS
  rosbuild/scripts/genmsg_cpp.py
  rosbuild/scripts/gensrv_cpp.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)
install(FILES
  rosbuild/scripts/msg_gen.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)

add_subdirectory(test)
add_subdirectory(perf)
add_subdirectory(test_serialization)
