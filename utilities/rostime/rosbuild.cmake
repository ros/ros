if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set the default path for built executables to the "bin" directory
#set the default path for built libraries to the "lib" directory
rosbuild_add_boost_directories()
rosbuild_add_library(${PROJECT_NAME} src/time.cpp src/rate.cpp src/duration.cpp)
rosbuild_link_boost(${PROJECT_NAME} thread)
if(NOT APPLE OR WIN32)
target_link_libraries(${PROJECT_NAME} rt)
endif()

