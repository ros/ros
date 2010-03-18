include(CheckIncludeFiles)
include(CheckFunctionExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)

# Output test results to config.h
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_boost_directories()

rosbuild_add_library(ros 
                    common.cpp 
                    poll_set.cpp 
                    header.cpp
                    connection.cpp
                    transport/transport_tcp.cpp 
                    transport/transport_udp.cpp
                    internal_timer_manager.cpp
                    this_node.cpp 
                    names.cpp 
                    master.cpp 
                    param.cpp 
                    network.cpp 
                    file_log.cpp 
                    init.cpp 
                    xmlrpc_manager.cpp 
                    poll_manager.cpp 
                    connection_manager.cpp
                    topic_manager.cpp
                    service_manager.cpp
                    message_deserializer.cpp
                    subscription_queue.cpp
                    publisher_link.cpp
                    transport_publisher_link.cpp
                    intraprocess_publisher_link.cpp
                    publication.cpp
                    subscription.cpp
                    subscriber_link.cpp
                    transport_subscriber_link.cpp
                    intraprocess_subscriber_link.cpp 
                    service_publication.cpp
                    service_server_link.cpp
                    service_client_link.cpp
                    service.cpp
                    topic.cpp
                    rosout_appender.cpp
                    callback_queue.cpp
                    spinner.cpp
                    node_handle.cpp
                    publisher.cpp
                    subscriber.cpp
                    service_server.cpp
                    service_client.cpp
                    single_subscriber_publisher.cpp
                    timer.cpp
                    wall_timer.cpp
                    )
target_link_libraries(ros)
rosbuild_link_boost(ros thread signals filesystem system)
