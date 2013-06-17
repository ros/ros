###############################################################################
# Internal macros below

macro(_rosbuild_warn)
  message("[rosbuild] WARNING: " ${ARGV})
endmacro(_rosbuild_warn)

macro(_rosbuild_warn_deprecate_rospack_prefix name)
  string(REPLACE rospack rosbuild new_name ${name})
  message("[rosbuild] WARNING: ${name} is deprecated; please use ${new_name} instead")
endmacro(_rosbuild_warn_deprecate_rospack_prefix)

macro(_rosbuild_warn_deprecate_no_prefix name)
  message("[rosbuild] WARNING: ${name} is deprecated; please use rosbuild_${name} instead")
endmacro(_rosbuild_warn_deprecate_no_prefix)

# look up python interpreter, store in ${PYTHON_EXECUTABLE}
find_package(PythonInterp)

###############################################################################
# Macro to turn a list into a string (why doesn't CMake have this
# built-in?)
macro(_rosbuild_list_to_string _string _list)
    set(${_string})
    foreach(_item ${_list})
        string(LENGTH "${${_string}}" _len)
        if(${_len} GREATER 0)
          set(${_string} "${${_string}} ${_item}")
        else(${_len} GREATER 0)
          set(${_string} "${_item}")
        endif(${_len} GREATER 0)
    endforeach(_item)
endmacro(_rosbuild_list_to_string)

###############################################################################
# Macro to dequote a string, in order to properly construct a command line.
# There must be an easier way to do this.
macro(_rosbuild_dequote_string _out _in)
  set(${_out})
  string(REGEX REPLACE " " ";" tmp "${_in}")
  foreach(_item ${tmp})
    string(LENGTH "${${_out}}" _len)
    if(${_len} GREATER 0)
      set(${_out} ${${_out}} ${_item})
    else(${_len} GREATER 0)
      set(${_out} ${_item})
    endif(${_len} GREATER 0)
  endforeach(_item)
endmacro(_rosbuild_dequote_string)


# list(FIND) was introduced after cmake 2.4.6, so we write our own
macro(_rosbuild_list_find _list _item _idx)
    set(${_idx} -1)
    list(LENGTH ${_list} _len)
    math(EXPR _total "${_len} - 1")
    foreach(_i RANGE ${_total})
      list(GET ${_list} ${_i} _it)
      if(_it STREQUAL ${_item})
        set(${_idx} ${_i})
      endif(_it STREQUAL ${_item})
    endforeach(_i)
endmacro(_rosbuild_list_find)

# Check validity of manifest.xml, to avoid esoteric build errors
macro(_rosbuild_check_manifest)
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import roslib.manifest; roslib.manifest.parse_file('manifest.xml')"
    OUTPUT_VARIABLE _manifest_error
    ERROR_VARIABLE _manifest_error
    RESULT_VARIABLE _manifest_failed
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(_manifest_failed)
    message("[rosbuild] Error from syntax check of ${PROJECT_NAME}/manifest.xml")
    message("${_manifest_error}")
    message(FATAL_ERROR "[rosbuild] Syntax check of ${PROJECT_NAME}/manifest.xml failed; aborting")
  endif(_manifest_failed)


endmacro(_rosbuild_check_manifest)

# Check that the directory where we're building is also where rospack
# thinks that the package lives, to avoid esoteric build errors.
macro(_rosbuild_check_package_location)
  # Ask rospack where our package is
  rosbuild_find_ros_package(${PROJECT_NAME})
  # Compare to where we are
  execute_process(
    COMMAND $ENV{ROS_ROOT}/core/rosbuild/bin/check_same_directories.py ${${PROJECT_NAME}_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}
    OUTPUT_VARIABLE _rosbuild_check_package_location_error
    ERROR_VARIABLE _rosbuild_check_package_location_error
    RESULT_VARIABLE _rosbuild_check_package_location_failed
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(_rosbuild_check_package_location_failed)
    message("[rosbuild] Error from directory check: $ENV{ROS_ROOT}/core/rosbuild/bin/check_same_directories.py ${${PROJECT_NAME}_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}")
    message("${_rosbuild_check_package_location_failed}")
    message("${_rosbuild_check_package_location_error}")
    message(FATAL_ERROR "[rosbuild] rospack found package \"${PROJECT_NAME}\" at \"${${PROJECT_NAME}_PACKAGE_PATH}\", but the current directory is \"${PROJECT_SOURCE_DIR}\".  You should double-check your ROS_PACKAGE_PATH to ensure that packages are found in the correct precedence order.")
  endif(_rosbuild_check_package_location_failed)
endmacro(_rosbuild_check_package_location)

# helper function to register check that results were generated (#580)
macro(_rosbuild_check_rostest_xml_result test_name test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${ROSUNIT_SCRIPTS_DIR}/check_test_ran.py ${test_file}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test-results-run)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test-results-run ${test_name}_result)
endmacro(_rosbuild_check_rostest_xml_result test_name)

macro(_rosbuild_add_gtest exe)
  # Look for optional TIMEOUT argument, #2645
  cmake_parse_arguments(_gtest "" "TIMEOUT" "" ${ARGN})
  if(NOT _gtest_TIMEOUT)
    set(_gtest_TIMEOUT 60.0)
  endif(NOT _gtest_TIMEOUT)

  # Create the program, with basic + gtest build flags
  rosbuild_add_executable(${exe} EXCLUDE_FROM_ALL ${_gtest_UNPARSED_ARGUMENTS})
  rosbuild_add_gtest_build_flags(${exe})

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})


  # Create target for this test
  # We use rosunit to call the executable to get process control, #1629, #3112
  # But don't depend on the gtest executable if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_custom_target(test_${_testname}
                      COMMAND ${ROSUNIT_EXE} --name=${_testname} --time-limit=${_gtest_TIMEOUT} ${EXECUTABLE_OUTPUT_PATH}/${exe}
                      DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${exe}
                      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                      VERBATIM)
  else(NOT rosbuild_test_nobuild)
    add_custom_target(test_${_testname}
                      COMMAND ${ROSUNIT_EXE} --name=${_testname} --time-limit=${_gtest_TIMEOUT} ${EXECUTABLE_OUTPUT_PATH}/${exe}
                      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                      VERBATIM)
  endif(NOT rosbuild_test_nobuild)
  # Don't register to check xml output here, because we may have gotten
  # here through registration of a future test.  Eventually, we should pass
  # in the overriding target (e.g., test-results vs. test-future-results).
  # For now, we call _rosbuild_check_rostest_xml_result() in rosbuild_add_gtest() instead.
  #_rosbuild_check_rostest_xml_result(test_${_testname} ${rosbuild_test_results_dir}/${PROJECT_NAME}/${_testname}.xml)

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # Make sure all test programs are built before running this test
  # but not if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_dependencies(test_${_testname} tests)
  endif(NOT rosbuild_test_nobuild)

endmacro(_rosbuild_add_gtest)

# helper function to register check that results were generated (#580)
# this one specifically targets rostest. rostest requires different
# arguments as cmake doesn't know the name of the output file
macro(_rosbuild_check_rostest_result test_name test_pkg test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${ROSUNIT_SCRIPTS_DIR}/check_test_ran.py --rostest ${test_pkg} ${test_file}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test-results-run)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test-results-run ${test_name}_result)
endmacro(_rosbuild_check_rostest_result test_name)

macro(_rosbuild_add_rostest file)

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} ${PROJECT_SOURCE_DIR} /)
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find rostest file \"${file}\"")
  endif(NOT _file_name)

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})

  # Create target for this test
  add_custom_target(rostest_${_testname}
                    COMMAND ${ARGN} rostest ${file}
                    DEPENDS ${file}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    VERBATIM)

  # Make sure all test programs are built before running this test
  # but not if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_dependencies(rostest_${_testname} tests)
  endif(NOT rosbuild_test_nobuild)

  # rostest-check-results will do the magic of fixing an incorrect file extension
  # Don't register to check rostest output here, because we may have gotten
  # here through registration of a future test.  Eventually, we should pass
  # in the overriding target (e.g., test-results vs. test-future-results).
  # For now, we call _rosbuild_check_rostest_xml_result() in
  # rosbuild_add_rostest()
  # and rosbuild_add_rostest_future() instead.
  #_rosbuild_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(_rosbuild_add_rostest)

macro(_rosbuild_add_pyunit file)
  # Look for optional TIMEOUT argument, #2645
  cmake_parse_arguments(_pyunit "" "TIMEOUT" "" ${ARGN})
  if(NOT _pyunit_TIMEOUT)
    set(_pyunit_TIMEOUT 60.0)
  endif(NOT _pyunit_TIMEOUT)

  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} ${PROJECT_SOURCE_DIR} /)
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find pyunit file \"${file}\"")
  endif(NOT _file_name)

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})

  # We look for ROS_TEST_COVERAGE=1
  # to indicate that coverage reports are being requested.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg "--cov")
  else("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    set(_covarg)
  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")

  # Create target for this test
  # We use rostest to call the executable to get process control, #1629
  add_custom_target(pyunit_${_testname}
                    COMMAND ${ROSUNIT_EXE} --name=${_testname} --time-limit=${_pyunit_TIMEOUT} -- ${file} ${_covarg}
                    DEPENDS ${file}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    VERBATIM)

  # Make sure all test programs are built before running this test
  # but not if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_dependencies(pyunit_${_testname} tests)
  endif(NOT rosbuild_test_nobuild)

endmacro(_rosbuild_add_pyunit)

# Actual signature:
#  _rosbuild_add_roslaunch_check targetname file var=val var=val...
macro(_rosbuild_add_roslaunch_check targetname file)
  # Check that the file exists, #1621
  set(_file_name _file_name-NOTFOUND)
  find_file(_file_name ${file} ${CMAKE_CURRENT_SOURCE_DIR} /)
  if(NOT _file_name)
    message(FATAL_ERROR "Can't find roslaunch file or directory \"${file}\"")
  endif(NOT _file_name)

  # Find roslaunch
  rosbuild_invoke_rospack("" roslaunch path find roslaunch)

  # Create target for this test.
  add_custom_target(${targetname}
                    COMMAND ${roslaunch_path}/scripts/roslaunch-check -o ${rosbuild_test_results_dir}/${PROJECT_NAME}/TEST-${targetname}.xml ${file} ${ARGN}
                    DEPENDS ${file}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    VERBATIM)

  # Make sure all test programs are built before running this test
  # but not if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_dependencies(${targetname} tests)
  endif(NOT rosbuild_test_nobuild)

endmacro(_rosbuild_add_roslaunch_check)

macro(_rosbuild_wget_and_build tarball tarball_url tarball_dir unpack_cmd configure_cmd make_cmd install_cmd)
  find_package(Wget REQUIRED)

  _rosbuild_dequote_string(_unpack_cmd ${unpack_cmd})
  _rosbuild_dequote_string(_configure_cmd ${configure_cmd})
  _rosbuild_dequote_string(_make_cmd ${make_cmd})
  _rosbuild_dequote_string(_install_cmd ${install_cmd})

  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/${tarball}
                     COMMAND ${WGET_EXECUTABLE} ${tarball_url} -O ${tarball}
		     VERBATIM)

  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/${tarball_dir}
                     COMMAND ${_unpack_cmd} ${tarball}
                     COMMAND touch ${tarball_dir}
		     DEPENDS ${PROJECT_BINARY_DIR}/${tarball}
		     VERBATIM)

  add_custom_command(OUTPUT ${PROJECT_BINARY_DIR}/installed
                     COMMAND cmake -E chdir ${PROJECT_BINARY_DIR}/${tarball_dir} ${_configure_cmd}
                     COMMAND cmake -E chdir ${PROJECT_BINARY_DIR}/${tarball_dir} ${_make_cmd}
                     COMMAND cmake -E chdir ${PROJECT_BINARY_DIR}/${tarball_dir} ${_install_cmd}
                     COMMAND touch ${PROJECT_BINARY_DIR}/installed
		     DEPENDS ${PROJECT_BINARY_DIR}/${tarball_dir}
                     VERBATIM)

  add_custom_target(fetch_and_build ALL
                    DEPENDS ${PROJECT_BINARY_DIR}/installed)
endmacro(_rosbuild_wget_and_build)

macro(_rosbuild_add_library lib libname type)

  add_library(${lib} ${type} ${ARGN})

  if(${type} STREQUAL STATIC)
    # Set output name to be the same as shared lib (may not work on Windows)
    set_target_properties(${lib} PROPERTIES OUTPUT_NAME ${libname})
    # Also add -fPIC, because CMake leaves it out when building static
    # libs, even though it's necessary on 64-bit machines for linking this
    # lib against shared libs downstream. The only exception is mingw gcc
    # which doesn't specifically need to worry about building
    # position independant libs.
    if(NOT MINGW)
        rosbuild_add_compile_flags(${lib} -fPIC)
    endif()
  endif(${type} STREQUAL STATIC)

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies
  get_target_property(_srclist ${lib} SOURCES)
  foreach(_src ${_srclist})
    set(_file_name _file_name-NOTFOUND)
    find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
    if(NOT _file_name)
      message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
      set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
    endif(NOT _file_name)
    add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST})
  endforeach(_src)

  # Prevent deletion of existing lib of same name
  set_target_properties(${lib} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
  # Attach compile and link flags
  rosbuild_add_compile_flags(${lib} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rosbuild_add_link_flags(${lib} ${${PROJECT_NAME}_LDFLAGS_OTHER})
  # Link lib against dependent libs
  target_link_libraries(${lib} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rosbuild_add_compile_flags(${lib} ${ROS_COMPILE_FLAGS})
  rosbuild_add_link_flags(${lib} ${ROS_LINK_FLAGS})

  # Make sure to do any prebuild work (e.g., msg/srv generation) before
  # building this target.
  add_dependencies(${lib} rosbuild_precompile)
endmacro(_rosbuild_add_library)

macro(_rosbuild_get_clock var)
  execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import time, sys; sys.stdout.write(str(time.time()));"
    OUTPUT_VARIABLE ${var}
    ERROR_VARIABLE _time_error
    RESULT_VARIABLE _time_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(_time_failed)
    message("[rosbuild] Error from calling to Python to get system time:")
    message("${_time_error}")
    message(FATAL_ERROR "[rosbuild] Failed to get system time; aborting")
  endif(_time_failed)
endmacro(_rosbuild_get_clock var)

macro(_rosbuild_cmakelist_to_pylist _cmakelist _pylist)
    # Convert a CMake list into a Python list
    set(_pyl "[")
    foreach(_f ${_cmakelist})
      set(_pyl "${_pyl} '${_f}',")
    endforeach(_f)
    set(_pyl "${_pyl}]")
    set(${_pylist} "${_pyl}")
endmacro(_rosbuild_cmakelist_to_pylist _cmakelist _pylist)

macro(_rosbuild_compare_manifests var _t _c _m)
  if("${_t}" STREQUAL "")
    # No time was given, so it's too old
    set(${var} 1)
  else("${_t}" STREQUAL "")
    _rosbuild_cmakelist_to_pylist("${_m}" _pylist)
    _rosbuild_cmakelist_to_pylist("${_c}" _cached_pylist)

    # Call Python to compare the provided time to the latest mtime on all
    # the files
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import os, sys; sys.stdout.write('1' if set(${_pylist}) != set(${_cached_pylist}) or ${_t} < max(os.stat(f).st_mtime for f in ${_pylist}) else '0');"
      OUTPUT_VARIABLE ${var}
      ERROR_VARIABLE _mtime_error
      RESULT_VARIABLE _mtime_failed
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(_mtime_failed)
      message("[rosbuild] Error from calling to Python to get latest mtime:")
      message("${_mtime_error}")
      message(FATAL_ERROR "[rosbuild] Failed to get latest mtime; aborting")
    endif(_mtime_failed)
  endif("${_t}" STREQUAL "")
endmacro(_rosbuild_compare_manifests var _t)

# parse_arguments() taken from
# http://www.itk.org/Wiki/CMakeMacroParseArguments
MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})
    SET(larg_names ${arg_names})
    LIST(FIND larg_names "${arg}" is_arg_name)
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})
      LIST(FIND loption_names "${arg}" is_option)
      IF (is_option GREATER -1)
             SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
             SET(current_arg_list ${current_arg_list} ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

# Internal macros above
###############################################################################

