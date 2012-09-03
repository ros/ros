cmake_minimum_required(VERSION 2.8)
project(rosbash)
find_package(catkin)
catkin_project(rosbash)

install(FILES rosbash rostcsh roszsh
  DESTINATION ${PROJECT_SHARE_DESTINATION})
install(PROGRAMS scripts/rosrun
  DESTINATION ${GLOBAL_BIN_DESTINATION})

message(STATUS "   Making toplevel forward script for bash script rosrun")
set(BASH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun)
configure_file(${catkin_EXTRAS_DIR}/templates/script.bash.in
  ${catkin_BUILD_PREFIX}/bin/rosrun
  @ONLY)

catkin_add_env_hooks(15.rosbash SHELLS bash tcsh zsh)
