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
#uncomment if you have defined messages
#uncomment if you have defined services
rosbuild_add_boost_directories()
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/intra.cpp src/inter.cpp)
rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(intra_suite src/intra_suite.cpp)
target_link_libraries(intra_suite ${PROJECT_NAME})

