cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(ros)
find_package(Boost REQUIRED COMPONENTS thread)

# Must call catkin_python_setup() early, to ensure that things like
# rosunit are available in build-space, and can be found by things like
# add_gtest(), which is called from within lower-level CMakeLists.txts.
catkin_python_setup()

catkin_add_env_hooks(10.ros SHELLS bat sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

foreach(subdir
    tools/rosunit
    core/roslib
    tools/rosbash
    )
  add_subdirectory(${subdir})
endforeach()

# nosetests are declared here to make them visible to the build system
# (e.g., for Jenkins to invoke) but keep them from polluting the pristine
# Python packages.
find_package(catkin COMPONENTS rosunit)
add_nosetests(core/roslib/test)
add_nosetests(tools/rosmake/test)
add_nosetests(tools/rosunit/test)
add_nosetests(tools/rosclean/test)
add_nosetests(tools/roscreate/test)

# INSTALLATION
install(FILES config/rosconsole.config
  DESTINATION ${PROJECT_SHARE_DESTINATION}/config)

# setup our backwards-compatibility ROS_ROOT
install(DIRECTORY core/rosbuild/
  DESTINATION ${PROJECT_SHARE_DESTINATION}/core/rosbuild
  PATTERN ".svn" EXCLUDE
  PATTERN "bin" EXCLUDE
  )
install(PROGRAMS
  core/rosbuild/bin/check_same_directories.py
  core/rosbuild/bin/download_checkmd5.py
  DESTINATION ${PROJECT_SHARE_DESTINATION}/core/rosbuild/bin)
install(DIRECTORY core/mk/
  DESTINATION ${PROJECT_SHARE_DESTINATION}/core/mk
  PATTERN ".svn" EXCLUDE
  )
install(FILES core/roslang/cmake/roslang.cmake
  DESTINATION share/roslang/cmake)

install(DIRECTORY tools/roscreate/templates/
  DESTINATION share/roscreate/templates
  PATTERN ".svn" EXCLUDE)
