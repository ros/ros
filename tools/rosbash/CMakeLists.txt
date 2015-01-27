cmake_minimum_required(VERSION 2.8.3)
project(rosbash)
find_package(catkin)
catkin_package()

install(FILES rosbash rosfish rostcsh roszsh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/rosrun
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# making toplevel forward script for bash script rosrun
set(BASH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun)
configure_file(${catkin_EXTRAS_DIR}/templates/script.bash.in
  ${CATKIN_DEVEL_PREFIX}/bin/rosrun
  @ONLY)

catkin_add_env_hooks(15.rosbash SHELLS bash fish tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
