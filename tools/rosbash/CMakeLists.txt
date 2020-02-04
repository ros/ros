cmake_minimum_required(VERSION 3.0.2)
project(rosbash)
find_package(catkin)
catkin_package()

install(FILES rosbash rosfish rostcsh roszsh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if (WIN32)
  install(PROGRAMS scripts/rosrun.bat scripts/roscp.bat
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

  install(PROGRAMS scripts/rosfindpath.py scripts/roscd.bat scripts/rosls.bat
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
else()
  install(PROGRAMS scripts/rosrun
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif()

# making toplevel forward script for bash script rosrun
if(NOT WIN32)
  set(BASH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun)
  configure_file(${catkin_EXTRAS_DIR}/templates/script.bash.in
    ${CATKIN_DEVEL_PREFIX}/bin/rosrun
    @ONLY)
else()
  set(BAT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun.bat)
  configure_file(${catkin_EXTRAS_DIR}/templates/script.bat.in
    ${CATKIN_DEVEL_PREFIX}/bin/rosrun.bat
    @ONLY)
endif()

catkin_add_env_hooks(15.rosbash SHELLS bash fish tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
