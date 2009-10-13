cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()
set(ROS_BUILD_TYPE Debug)

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "rosoct install prefix" FORCE )
endif()

include(${CMAKE_SOURCE_DIR}/FindOctave.cmake)
if(NOT OCTAVE_FOUND)
  message(FATAL_ERROR "Octave installation not found")
endif(NOT OCTAVE_FOUND)

rosbuild_add_boost_directories()

include_directories(${OCTAVE_INCLUDE_DIRS})
link_directories(${OCTAVE_LINK_DIRS})
rosbuild_add_library(rosoct rosoct.cpp)
rosbuild_add_compile_flags(rosoct "${OCTAVE_CXXFLAGS}")
rosbuild_add_link_flags(rosoct "${OCTAVE_LINK_FLAGS}")
target_link_libraries(rosoct ${OCTAVE_LIBRARY} ${COIN_LIBRARY})
rosbuild_link_boost(rosoct thread)

set_target_properties(rosoct PROPERTIES PREFIX "" SUFFIX ".mex")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/octave)

## custom compiling doesn't detect dependency changes well
# find_ros_package(roscpp_sessions)
# _rospack_invoke(rosoct rosoct cflags --lang=cpp --attrib=cflags export)
# _rospack_invoke(rosoct rosoct lflags --lang=cpp --attrib=lflags export)

# add_custom_command(
#   OUTPUT ${OCTAVE_MEX}
#   COMMAND ${MKOCTFILE}
#   ARGS --mex ${ROS_COMPILE_FLAGS} ${ROS_LINK_FLAGS} ${rosoct_cflags} ${rosoct_lflags} -o \"${OCTAVE_MEX}\" \"${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp\"
#   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp ${roscpp_sessions_PACKAGE_PATH}/include/ros/session.h
#   )
# add_custom_target(rosoct_octave ALL DEPENDS ${OCTAVE_MEX})

rosbuild_gensrv()
rosbuild_genmsg()

#add_subdirectory(test EXCLUDE_FROM_ALL)
