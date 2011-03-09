if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

rosbuild_init()

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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/debug.cpp)