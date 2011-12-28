file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/rosmaster)
  execute_process(COMMAND /bin/ln -s ${CMAKE_CURRENT_SOURCE_DIR}/rosmaster ${CMAKE_BINARY_DIR}/bin/rosmaster)
endif()