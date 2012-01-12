cmake_minimum_required(VERSION 2.8)
project(rosbash)
install(FILES rosbash rostcsh roszsh
        DESTINATION share/rosbash)
install(PROGRAMS scripts/rosrun
        DESTINATION bin)
if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/rosrun)
  message(STATUS "   Making toplevel symlink for bash script rosrun")
  execute_process(COMMAND /bin/ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosrun ${CMAKE_BINARY_DIR}/bin/rosrun)
endif()

catkin_add_env_hooks(15.rosbash
  SHELLS bash zsh
  )
