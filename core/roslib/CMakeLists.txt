cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
rosbuild_init()

# To get system info used in time library
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
if (HAVE_EXECINFO_H)
  add_definitions(-DHAVE_EXECINFO_H=1)
endif(HAVE_EXECINFO_H)

# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include <cxxabi.h> \n int main(int argc, char** argv) {}" HAVE_CXXABI_H)
if (HAVE_CXXABI_H)
  add_definitions(-DHAVE_CXXABI_H=1)
endif()

CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)
if (HAVE_GLIBC_BACKTRACE)
  add_definitions(-DHAVE_GLIBC_BACKTRACE)
endif(HAVE_GLIBC_BACKTRACE)

rosbuild_add_boost_directories()

rosbuild_add_library(roslib src/time.cpp src/rate.cpp src/package.cpp src/debug.cpp src/serialization.cpp)
rosbuild_link_boost(roslib thread)

if (NOT APPLE)
target_link_libraries(roslib rt)
endif (NOT APPLE)
