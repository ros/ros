cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(include)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/malloc.cpp src/free_list.cpp src/publisher.cpp src/init.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_gtest(test_object_pool test/test_object_pool.cpp)
target_link_libraries(test_object_pool ${PROJECT_NAME})

rosbuild_add_gtest(test_freelist test/test_freelist.cpp)
target_link_libraries(test_freelist ${PROJECT_NAME})

rosbuild_add_gtest(test_aligned_alloc test/test_aligned_alloc.cpp)
target_link_libraries(test_aligned_alloc ${PROJECT_NAME})

rosbuild_add_gtest(test_publisher test/test_publisher.cpp)
target_link_libraries(test_publisher ${PROJECT_NAME})

rosbuild_add_gtest(test_malloc_wrappers test/test_malloc_wrappers.cpp)
target_link_libraries(test_malloc_wrappers ${PROJECT_NAME})