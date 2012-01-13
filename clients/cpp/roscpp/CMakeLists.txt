project(roscpp)

find_package(ROS COMPONENTS
  catkin genmsg
  cpp_common rostime rosconsole roscpp_serialization
  roscpp_traits std_msgs rosgraph_msgs xmlrpcpp
  )
include_directories(${ROS_INCLUDE_DIRS})

find_package(Boost COMPONENTS signals filesystem system)

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

add_library(roscpp SHARED
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

add_dependencies(roscpp roscpp_gencpp rosgraph_msgs_gencpp)

target_link_libraries(roscpp
  ${ROS_LIBRARIES}
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

generate_messages()

#explicitly install library and includes
install(TARGETS roscpp
  DESTINATION lib
 )
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/roscpp.cmake
        DESTINATION share/roscpp/rosbuild)
install(PROGRAMS 
        rosbuild/scripts/genmsg_cpp.py
        rosbuild/scripts/gensrv_cpp.py
        DESTINATION share/roscpp/rosbuild/scripts)
install(FILES 
        rosbuild/scripts/msg_gen.py
        DESTINATION share/roscpp/rosbuild/scripts)

catkin_project(roscpp
  LIBRARIES roscpp
  INCLUDE_DIRS include
  DEPENDS roscpp_traits roscpp_serialization cpp_common std_msgs rosgraph_msgs #EAR are these needed?  doesn't the stack.yaml declare these?
  )

message("TODO: add roscpp's tests back in")
