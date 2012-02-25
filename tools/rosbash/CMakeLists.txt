cmake_minimum_required(VERSION 2.8)
project(rosbash)
install(FILES rosbash rostcsh roszsh
        DESTINATION share/rosbash)
install(PROGRAMS scripts/rosrun
        DESTINATION bin)

message(STATUS "   Making toplevel forward script for bash script rosrun")
set(BASH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun)
configure_file(${catkin_EXTRAS_DIR}/templates/script.bash.in
  ${CMAKE_BINARY_DIR}/bin/rosrun
  @ONLY)

catkin_add_env_hooks(15.rosbash
  SHELLS bash zsh
  )
