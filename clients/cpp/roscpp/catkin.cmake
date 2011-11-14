project(roscpp)

find_package(catkin)
find_package(genmsg)

#find_package(ROS COMPONENTS cpp_common rostime rosconsole roscpp_serialization roscpp_traits std_msgs rosgraph_msgs XmlRpc)
#inclure_directories(${ROS}_INCLUDE_DIRS)

find_package(cpp_common)
include_directories(${cpp_common_INCLUDE_DIRS})

find_package(rostime)
include_directories(${rostime_INCLUDE_DIRS})

find_package(rosconsole)
include_directories(${rosconsole_INCLUDE_DIRS})

find_package(roscpp_serialization)
include_directories(${roscpp_serialization_INCLUDE_DIRS})

find_package(roscpp_traits)
include_directories(${roscpp_traits_INCLUDE_DIRS})

find_package(std_msgs)
include_directories(${std_msgs_INCLUDE_DIRS})

find_package(rosgraph_msgs)
include_directories(${rosgraph_msgs_INCLUDE_DIRS})

find_package(XmlRpc)
include_directories(${XmlRpc_INCLUDE_DIRS})

find_package(Boost COMPONENTS signals filesystem)

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

target_link_libraries(${PROJECT_NAME}
  ${roscpp_serialization_LIBRARIES}
  ${rostime_LIBRARIES}
  ${XmlRpc_LIBRARIES}
  ${rosconsole_LIBRARIES}
  ${Boost_LIBRARIES})

generate_msgs(${PROJECT_NAME}
  PATH msg
  MESSAGES msg/Logger.msg
  SERVICES srv/Empty.srv srv/GetLoggers.srv srv/SetLoggerLevel.srv
  )

install_cmake_infrastructure(roscpp
  VERSION 0.0.1
  LIBRARIES roscpp
  INCLUDE_DIRS include
  )
