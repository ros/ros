
cmake_minimum_required(VERSION 2.4.6)

#
#  Catkin-compat thunks
#
cmake_policy(SET CMP0011 OLD)

macro(rosbuild_catkinize)
  if(CATKIN)
    message(STATUS "thunking from rosbuild in ${CMAKE_CURRENT_SOURCE_DIR}")
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/catkin.cmake)
      include(catkin.cmake)
    else()
      message(STATUS "warning: Directory ${CMAKE_CURRENT_SOURCE_DIR} contains rosbuild_catkinize but no catkin.cmake")
    endif()
    return()
  endif()
endmacro()

if(CATKIN)
  return()
endif()

# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
  # Logical target names must be globally unique.
  cmake_policy(SET CMP0002 OLD)
  # Libraries linked via full path no longer produce linker search paths.
  cmake_policy(SET CMP0003 OLD)
  # Preprocessor definition values are now escaped automatically.
  cmake_policy(SET CMP0005 OLD)
  if(POLICY CMP0011)
    # Included scripts do automatic cmake_policy PUSH and POP.
    cmake_policy(SET CMP0011 OLD)
  endif(POLICY CMP0011)
endif(COMMAND cmake_policy)

set(CMAKE_OSX_ARCHITECTURES "x86_64")

##############################################################################
# First things first: we must have rospack.
find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable")
if (NOT ROSPACK_EXE)
  message(FATAL_ERROR "Couldn't find rospack. Please run 'make' in $ROS_ROOT")
endif(NOT ROSPACK_EXE)
##############################################################################

# Load private macros (not to be used externally)
include($ENV{ROS_ROOT}/core/rosbuild/private.cmake)
# Load public macros (developer's API)
include($ENV{ROS_ROOT}/core/rosbuild/public.cmake)
