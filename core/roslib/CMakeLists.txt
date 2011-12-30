cmake_minimum_required(VERSION 2.8)
project(roslib)
find_package(catkin)

install_cmake_infrastructure(roslib
  VERSION 0.0.1
  PYTHONPATH src
  )

enable_python(roslib)

# Avoid a boost warning that pops up when using msvc compiler
if(MSVC)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif()
rosbuild_add_boost_directories()
rosbuild_add_library(roslib src/package.cpp)
rosbuild_link_boost(roslib thread)

if(NOT (APPLE OR WIN32 OR MINGW))
  target_link_libraries(roslib rt)
endif()
