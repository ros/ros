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

# list(REMOVE_DUPLICATES) was introduced in cmake 2.6, so we write our own
macro(_rosbuild_list_remove_duplicates _inlist _outlist)
  foreach(_item ${_inlist})
    #list(FIND ${_outlist} ${_item} _idx)
    _rosbuild_list_find(${_outlist} ${_item} _idx)
    if(${_idx} EQUAL -1)
      list(APPEND ${_outlist} ${_item})
    endif(${_idx} EQUAL -1)
  endforeach(_item)
endmacro(_rosbuild_list_remove_duplicates)

# Check validity of PYTHONPATH, to avoid esoteric build errors, #954.
macro(_rosbuild_check_pythonpath)
  if("$ENV{PYTHONPATH}" STREQUAL "")
    message("WARNING: PYTHONPATH is not set.  This is almost certainly wrong. Check the ROS installation instructions for details on setting PYTHONPATH.")
  else("$ENV{PYTHONPATH}" STREQUAL "")
    if(NOT $ENV{PYTHONPATH} MATCHES ".*roslib.*")
      message("WARNING: PYTHONPATH does not appear to contain roslib.  This is almost certainly wrong. Check the ROS installation instructions for details on setting PYTHONPATH.")
    endif(NOT $ENV{PYTHONPATH} MATCHES ".*roslib.*")
  endif("$ENV{PYTHONPATH}" STREQUAL "")
endmacro(_rosbuild_check_pythonpath)

# TODO: find a way to make this macro public; it's used in making
# distributions
macro(rosbuild _project)
  # Establish project name.  Retrieve it later as PROJECT_NAME.
  project(${_project})

  # Set up for packaging
  # TODO: get version from manifest
  #set(CPACK_PACKAGE_VERSION_MAJOR "0")
  #set(CPACK_PACKAGE_VERSION_MINOR "0")
  #set(CPACK_PACKAGE_VERSION_PATCH "1")
  set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
  if("${ARGN}" STREQUAL "")
    set(CPACK_PACKAGE_VERSION "latest")
  else("${ARGN}" STREQUAL "")
    set(CPACK_PACKAGE_VERSION "${ARGN}")
  endif("${ARGN}" STREQUAL "")
  set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CPACK_PACKAGE_VERSION}")
  set(CPACK_GENERATOR "TBZ2")
  # The CPACK_SOURCE_GENERATOR variable seems only to be obeyed in 2.6.
  # 2.4 seems to use CPACK_GENERATOR for both binary and source packages.
  set(CPACK_SOURCE_GENERATOR "TBZ2")
  # CPACK_SOURCE_IGNORE_FILES contains things we want to ignore when
  # building a source package.  We assume that the package was already
  # cleaned, so we don't need to ignore .a, .o, .so, etc.
  list(APPEND CPACK_SOURCE_IGNORE_FILES "/build/;/.svn/;.gitignore;.rosgcov_files;.build_version;build-failure;test-failure;rosmakeall-buildfailures-withcontext.txt;rosmakeall-profile;rosmakeall-buildfailures.txt;rosmakeall-testfailures.txt;rosmakeall-coverage.txt;/log/")
  message("in rosbuild:${CPACK_SOURCE_IGNORE_FILES}:")
  include(CPack)
endmacro(rosbuild)

macro(_rosbuild_add_gcov src exe)
  set(_gcov ${exe}_${_src}.gcov)
  string(REPLACE "/" "_" _targetname ${_gcov})
  add_custom_target(${_targetname} 
                    COMMAND rosgcov ${_src} ${PROJECT_BINARY_DIR}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  add_dependencies(${_targetname} test)
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(gcoverage-run)
  add_dependencies(gcoverage-run ${_targetname})
  file(APPEND ${PROJECT_SOURCE_DIR}/.rosgcov_files "${CMAKE_CURRENT_SOURCE_DIR} ${_src}\n")
endmacro(_rosbuild_add_gcov)

# helper function to register check that results were generated (#580)
macro(_rosbuild_check_rostest_xml_result test_name test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${rostest_path}/bin/rostest-check-results ${test_file}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-results-run)
  add_dependencies(test-results-run ${test_name}_result)	 
endmacro(_rosbuild_check_rostest_xml_result test_name)

macro(_rosbuild_add_gtest exe)

  # Create the program, with basic + gtest build flags
  rosbuild_add_executable(${exe} EXCLUDE_FROM_ALL ${ARGN})
  rosbuild_add_gtest_build_flags(${exe})

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})


  # Create target for this test
  add_custom_target(test_${_testname}
                    COMMAND ${PROJECT_SOURCE_DIR}/${exe} --gtest_output=xml:$ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}/${_testname}.xml
                    DEPENDS ${PROJECT_SOURCE_DIR}/${exe}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    VERBATIM)
  # Don't register to check xml output here, because we may have gotten
  # here through registration of a future test.  Eventually, we should pass
  # in the overriding target (e.g., test-results vs. test-future-results).
  # For now, we call _rosbuild_check_rostest_xml_result() in rosbuild_add_gtest() instead.
  #_rosbuild_check_rostest_xml_result(test_${_testname} $ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}/${_testname}.xml)

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # Make sure all test programs are built before running this test
  add_dependencies(test_${_testname} tests)

endmacro(_rosbuild_add_gtest)

# helper function to register check that results were generated (#580)
# this one specifically targets rostest. rostest requires different
# arguments as cmake doesn't know the name of the output file
macro(_rosbuild_check_rostest_result test_name test_pkg test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${rostest_path}/bin/rostest-check-results --rostest ${test_pkg} ${test_file}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-results-run)
  add_dependencies(test-results-run ${test_name}_result)	 
endmacro(_rosbuild_check_rostest_result test_name)

macro(_rosbuild_add_rostest file)

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})

  # Create target for this test
  add_custom_target(rostest_${_testname}
                    COMMAND ${ARGN} rostest ${file}
                    DEPENDS ${file}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    VERBATIM)

  # Make sure all test programs are built before running this test
  add_dependencies(rostest_${_testname} tests)

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

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${file})

  # Create target for this test
  add_custom_target(pyunit_${_testname}
                    COMMAND ${ARGN} python ${file}
                    DEPENDS ${file}
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                    VERBATIM)

  # Make sure all test programs are built before running this test
  add_dependencies(pyunit_${_testname} tests)

endmacro(_rosbuild_add_pyunit)

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

# Internal macros above
###############################################################################

