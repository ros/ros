file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/rosgraph)
  execute_process(COMMAND /bin/ln -s ${CMAKE_CURRENT_SOURCE_DIR}/rosgraph ${CMAKE_BINARY_DIR}/bin/rosgraph)
endif()
