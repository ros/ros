include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

include_directories(include ${PROJECT_SOURCE_DIR})

add_definitions(-DTIXML_USE_STL)
if(ROS_BUILD_STATIC_LIBS)
  add_definitions("-DROS_STATIC")
endif()

set(rospack_sources rospack.cpp 
  tinyxml-2.5.3/tinystr.cpp 
  tinyxml-2.5.3/tinyxml.cpp 
  tinyxml-2.5.3/tinyxmlparser.cpp 
  tinyxml-2.5.3/tinyxmlerror.cpp)

set(rosstack_sources rosstack.cpp)

rosbuild_add_library(rospack ${rospack_sources})
rosbuild_add_library(rosstack ${rosstack_sources})

rosbuild_add_executable(rospackexe main.cpp)
set_target_properties(rospackexe PROPERTIES OUTPUT_NAME rospack)
target_link_libraries(rosstack rospack)

rosbuild_add_executable(rosstackexe rosstack_main.cpp)
set_target_properties(rosstackexe PROPERTIES OUTPUT_NAME rosstack)
target_link_libraries(rospackexe rospack)
target_link_libraries(rosstackexe rosstack rospack)

install(TARGETS rospack rosstack rospackexe rosstackexe
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES include/rospack/rospack.h
        DESTINATION include/rospack)

