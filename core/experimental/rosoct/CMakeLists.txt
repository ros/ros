cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rosbuild_init()

set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

if(UNIX AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "rosoct install prefix" FORCE )
endif()

include(${CMAKE_SOURCE_DIR}/FindOctave.cmake)
if(NOT OCTAVE_FOUND)
  message(FATAL_ERROR "Octave installation not found")
endif(NOT OCTAVE_FOUND)

rospack_add_boost_directories()


include_directories(${OCTAVE_INCLUDE_DIRS})
link_directories(${OCTAVE_LINK_DIRS})
rospack_add_library(rosoct rosoct.cpp)
rospack_add_compile_flags(rosoct "${OCTAVE_CXXFLAGS}")
rospack_add_link_flags(rosoct "${OCTAVE_LINK_FLAGS}")
target_link_libraries(rosoct ${OCTAVE_LIBRARY} ${COIN_LIBRARY})
rospack_link_boost(rosoct thread)

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

gensrv()
genmsg()

add_subdirectory(test EXCLUDE_FROM_ALL)

#########
## MATLAB (if installed)
##########

#include(CheckLibraryExists)
#
#set(MATLAB MATLAB-NOTFOUND)
#if( WIN32 )
#  FIND_PROGRAM(MATLAB NAME "mex.bat" PATHS )
#else()
#  FIND_PROGRAM(MATLAB NAME "mex" PATHS )
#endif()
# 
#set(MEX_CXXFLAGS)
#if( CMAKE_COMPILER_IS_GNUC OR CMAKE_COMPILER_IS_GNUCXX )
#  if( NOT WIN32 )
#    # check for lapack
#    check_library_exists(lapack _init "" HAS_LAPACK_LIB)
#    if( HAS_LAPACK_LIB )
#      set(MEX_CXXFLAGS "${MEX_LIBS}")
#    endif()    
#  endif()
#else()
#  if( MSVC )
#    set(MEX_CXXFLAGS "ws2_32.lib")
#  endif()
#endif()
#
#if(MATLAB)
#  
#  # check if the mex file is actually matlab (can be confused with latex)
#  EXEC_PROGRAM(mex ARGS "-version" OUTPUT_VARIABLE MEX_TEST_OUT RETURN_VALUE MEX_TEST_RETURN)
#  #message(STATUS "mex result: ${MEX_TEST_RETURN}")
#  string(REGEX MATCH "MATLAB" IS_MATLAB "${MEX_TEST_OUT}")
#  
#  if( IS_MATLAB )
#    set(USE_MATLAB 1)
#  else()
#    set(USE_MATLAB)
#  endif()
#endif()
#
#find_ros_package(roscpp_sessions)
#_rospack_invoke(rosoct rosoct cflags export --lang=cpp --attrib=cflags)
#_rospack_invoke(rosoct rosoct lflags export --lang=cpp --attrib=lflags)
#
#if( USE_MATLAB )
#  message(STATUS "Matlab detected, compiling matlab mex files")
#  
#  if( DARWIN OR APPLE )
#    set(MEXEXT "mexmac")
#  elseif( UNIX )
#    if( IS_X86_64 )
#      set(MEXEXT "mexa64")
#    else()
#      set(MEXEXT "mexglx")
#    endif()
#  elseif( WIN64 )
#    set(MEXEXT "mexw64")
#  elseif( WIN32 OR CYGWIN OR WINDOWS )
#    set(MEXEXT "mexw32")
#  else()
#    set(MEXEXT "mex")
#  endif()
#  
#  set(MATLAB_MEX_OUT ${CMAKE_CURRENT_SOURCE_DIR}/matlab)
#  set(MATLAB_ROSOCT_MEX "rosoct.${MEXEXT}")
#  add_custom_command(
#    OUTPUT ${MATLAB_MEX_OUT}/${MATLAB_ROSOCT_MEX}
#    COMMAND mex
#    ARGS ${rosoct_cflags} ${rosoct_lflags} -outdir \"${MATLAB_MEX_OUT}\" -output \"${MATLAB_ROSOCT_MEX}\" \"${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp\"
#    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/rosoct.cpp
#    )
#  add_custom_target(rosoct_matlab ALL DEPENDS ${MATLAB_MEX_OUT}/${MATLAB_ROSOCT_MEX})
#endif()
