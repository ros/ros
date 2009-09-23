cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_INSTALL_PREFIX /tmp/rospack)
#set(CMAKE_INSTALL_RPATH_USE_LINK_RPATH true)
#set(CMAKE_SKIP_BUILD_RPATH true)
set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

include_directories(include ${PROJECT_SOURCE_DIR})

add_library(rospack SHARED rospack.cpp
                    tinyxml-2.5.3/tinystr.cpp
                    tinyxml-2.5.3/tinyxml.cpp
                    tinyxml-2.5.3/tinyxmlparser.cpp
                    tinyxml-2.5.3/tinyxmlerror.cpp)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)
add_executable(rospackexe main.cpp)
set_target_properties(rospackexe PROPERTIES OUTPUT_NAME rospack)
target_link_libraries(rospackexe rospack)

add_library(rosstack SHARED rosstack.cpp)
target_link_libraries(rosstack rospack)
add_executable(rosstackexe rosstack_main.cpp)
set_target_properties(rosstackexe PROPERTIES OUTPUT_NAME rosstack)
target_link_libraries(rosstackexe rosstack rospack)

install(TARGETS rospack rosstack rospackexe rosstackexe
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)
install(FILES include/rospack/rospack.h
        DESTINATION include/rospack)

# Prevent warnings about duplicate definition of the targets mentioned
# below
if(COMMAND cmake_policy)
  # Logical target names must be globally unique.
  cmake_policy(SET CMP0002 OLD)
endif(COMMAND cmake_policy)
# These targets might be called by rosmakeall
add_custom_target(test)
add_custom_target(tests)
add_custom_target(test-results)
add_custom_target(test-future)
add_custom_target(gcoverage)
