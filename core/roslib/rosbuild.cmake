include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

rosbuild_add_boost_directories()
rosbuild_add_library(roslib src/package.cpp)
rosbuild_link_boost(roslib thread)

if(NOT APPLE)
  target_link_libraries(roslib rt)
endif(NOT APPLE)

