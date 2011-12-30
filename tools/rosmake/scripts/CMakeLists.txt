file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(NOT EXISTS ${CMAKE_BINARY_DIR}/bin/rosmake)
  execute_process(COMMAND /bin/ln -s ${CMAKE_CURRENT_SOURCE_DIR}/rosmake ${CMAKE_BINARY_DIR}/bin/rosmake)
endif()
