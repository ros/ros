include(CheckIncludeFiles)
include(CheckFunctionExists)

# Not everybody has <ifaddrs.h> (e.g., embedded arm-linux)
CHECK_INCLUDE_FILES(ifaddrs.h HAVE_IFADDRS_H)
# Not everybody has trunc (e.g., Windows, embedded arm-linux)
CHECK_FUNCTION_EXISTS(trunc HAVE_TRUNC)

# Output test results to config.h
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

rospack_add_boost_directories()

rospack_add_library(ros 
                    common.cpp poll_set.cpp header.cpp 
                    connection.cpp transport/transport_tcp.cpp
                    publisher.cpp publication.cpp subscription.cpp subscriber.cpp 
                    service_server.cpp service_link.cpp service_client.cpp service.cpp
                    rosout_appender.cpp node.cpp
                    )
rospack_link_boost(ros thread signals)
