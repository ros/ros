cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rospack_add_boost_directories()

set(wxWidgets_USE_LIBS base core)

find_package(wxWidgets REQUIRED)

if(NOT wxWidgets_FOUND)
message(FATAL_ERROR "wxWidgets not found!  Aborting...")
endif(NOT wxWidgets_FOUND)

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/src)

# shared library (for C++)
rospack_add_library(${PROJECT_NAME} src/rxtools/topic_display.cpp
                                    src/rxtools/topic_display_generated.cpp
        				                    src/rxtools/topic_display_dialog.cpp
        				                    src/rxtools/rosout_generated.cpp 
                                    src/rxtools/rosout_panel.cpp
                                    src/rxtools/rosout_list_control.cpp
                                    src/rxtools/rosout_setup_dialog.cpp
                                    src/rxtools/logger_level_panel.cpp
                                    src/rxtools/init_roscpp.cpp)
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES})
rospack_link_boost(${PROJECT_NAME} thread regex)

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "export" "--lang=swig" "--attrib=flags")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})

# Find the wxswig executable
find_ros_package(wxswig)
set(WXSWIG_EXECUTABLE ${wxswig_PACKAGE_PATH}/bin/swig)

# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/src/rxtools/rxtools.i)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/lib/rxtoolscpp.py)

# Deal with CMake 2.4 vs. 2.6
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)   # CMake 2.6
  set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/src/rxtools/rosout_panel_swig_generated.cpp)
  set(SWIG_COMMAND ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module rxtoolscpp ${SWIG_INTERFACE_FILE})
else("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)    # CMake 2.4
  set(SWIG_OUTPUT_CPP_FILE src/rxtools/rosout_panel_swig_generated.cpp)
  set(SWIG_COMMAND ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ../${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module rxtoolscpp ${SWIG_INTERFACE_FILE})
endif("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" GREATER 2.4)

set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE} 
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${SWIG_COMMAND}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
                            src/rxtools/rosout_generated.h
                            src/rxtools/rosout_panel.h
                            src/rxtools/init_roscpp.h)
                            
# We add src/wx_rosout to the include list so that
# ogre_tools_swig_generated.cpp can find its headers.  This is part of a
# workaround for a deficiency in CMake 2.4's handling of generated sources.
include_directories(${PROJECT_SOURCE_DIR}/src/rxtools)
# shared library (with SWIG)
rospack_add_library(python_${PROJECT_NAME} ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(python_${PROJECT_NAME} ${PROJECT_NAME} ${PYTHON_LIBRARIES})

# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(python_${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}cpp
                      PREFIX "")
                      
#rxconsole binary
rospack_add_executable(rxconsole src/rxconsole/rxconsole.cpp)
target_link_libraries(rxconsole ${PROJECT_NAME})

#rxloggerlevel binary
rospack_add_executable(rxloggerlevel src/rxloggerlevel/rxloggerlevel.cpp)
target_link_libraries(rxloggerlevel ${PROJECT_NAME})

add_subdirectory(src/rxconsole)
add_subdirectory(src/rxloggerlevel)
