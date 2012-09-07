cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin REQUIRED)
catkin_stack()
catkin_project(ros)
find_package(Boost REQUIRED COMPONENTS thread)

# Must call catkin_python_setup() early, to ensure that things like
# rosunit are available in build-space, and can be found by things like
# catkin_add_gtest(), which is called from within lower-level CMakeLists.txts.
catkin_python_setup()

catkin_add_env_hooks(10.ros SHELLS bat sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

foreach(subdir
    core/roslang
    core/roslib
    tools/rosboost_cfg
    tools/rosbash
    tools/rosclean
    tools/roscreate
    tools/rosmake
    tools/rosunit
    )
  add_subdirectory(${subdir})
endforeach()

# INSTALLATION
install(FILES config/rosconsole.config
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/config)

# setup our backwards-compatibility ROS_ROOT
install(DIRECTORY core/rosbuild/
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/core/rosbuild
  PATTERN ".svn" EXCLUDE
  PATTERN "bin" EXCLUDE
  )
install(PROGRAMS
  core/rosbuild/bin/check_same_directories.py
  core/rosbuild/bin/download_checkmd5.py
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/core/rosbuild/bin)
install(DIRECTORY core/mk/
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/core/mk
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY tools/roscreate/templates/
  DESTINATION share/roscreate/templates
  PATTERN ".svn" EXCLUDE)
