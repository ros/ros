file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/rosnode)
  execute_process(COMMAND /bin/ln -s ${CMAKE_CURRENT_SOURCE_DIR}/rosnode ${CMAKE_BINARY_DIR}/bin/rosnode)
endif()
