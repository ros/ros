include(CMakeParseArguments)

macro(rosunit_initialize_tests)
@[if DEVELSPACE]@
  # binary and script in develspace
  set(ROSUNIT_SCRIPTS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
  set(ROSUNIT_EXE "${PYTHON_EXECUTABLE} ${ROSUNIT_SCRIPTS_DIR}/rosunit")
@[else]@
  # binary and script in installspace
  set(ROSUNIT_SCRIPTS_DIR "${rosunit_DIR}/../scripts")
  set(ROSUNIT_EXE "${PYTHON_EXECUTABLE} ${rosunit_DIR}/../../../@(CATKIN_GLOBAL_BIN_DESTINATION)/rosunit")
@[end if]@
endmacro()

rosunit_initialize_tests()

function(add_pyunit file)

  message(WARNING "add_pyunit() is deprecated.  For Python tests, use catkin_add_nosetests() instead.")

  # Look for optional TIMEOUT argument, #2645
  cmake_parse_arguments(_pyunit "" "TIMEOUT;WORKING_DIRECTORY" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif()

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  if(IS_ABSOLUTE ${file})
    set(_file_name ${file})
  else()
    find_file(_file_name ${file}
              PATHS ${CMAKE_CURRENT_SOURCE_DIR}
              NO_DEFAULT_PATH
              NO_CMAKE_FIND_ROOT_PATH)  # for cross-compilation.  thanks jeremy.
    if(NOT _file_name)
      message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
    endif()
  endif()

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else()
    set(_covarg)
  endif()

  # Create a legal test name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})
  # We use rostest to call the executable to get process control, #1629
  set(cmd "${ROSUNIT_EXE} --name=${_testname} --time-limit=${_pyunit_TIMEOUT} --package=${PROJECT_NAME} -- ${_file_name} ${_covarg}")
  catkin_run_tests_target("rosunit" ${_testname} "rosunit-${_testname}.xml" COMMAND ${cmd} WORKING_DIRECTORY ${_pyunit_WORKING_DIRECTORY})
endfunction()
