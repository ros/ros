project(test_roscreate)
find_package(catkin)

macro(append_project_cache CACHENAME)
  set(cachefile ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}.${CACHENAME})
  if(NOT ${PROJECT_NAME}_CACHE)
    file(WRITE ${cachefile} "#\n# This is ${cachefile}\n#\n")
    set(${PROJECT_NAME}_CACHE TRUE)
    # TODO: find catkin somehow
    set(runtests_path ${CMAKE_SOURCE_DIR}/catkin/scripts/runtests.py)
    add_custom_target(${PROJECT_NAME}_run_tests
                      COMMAND ${runtests_path} ${cachefile})
    add_custom_target(test)
    add_dependencies(test ${PROJECT_NAME}_run_tests)
  endif()

  # gotcha:  you need a newline or the message doesn't appear
  # in the cache.   pfft.
  file(APPEND ${cachefile} "${ARGN}\n")
endmacro()

function(add_pyunit file)
  # Look for optional TIMEOUT argument, #2645
  parse_arguments(_pyunit "TIMEOUT" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif(NOT _pyunit_TIMEOUT)

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} ${PROJECT_SOURCE_DIR})
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
  endif(NOT _file_name)

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg)
  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")

  # TODO: find rosunit somehow
  set(rosunit_path ${CMAKE_SOURCE_DIR}/ros/tools/rosunit)
  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})
  # We use rostest to call the executable to get process control, #1629
  append_project_cache(catkin-tests "${rosunit_path}/bin/rosunit --name=${_testname} --time-limit=${_pyunit_TIMEOUT} -- ${_file_name} ${_covarg}")
endfunction(add_pyunit)

add_pyunit(test/test_roscreate_stack.py)
