# Use this package to get add_file_dependencies()
include(AddFileDependencies)
# Used to check if a function exists
include(CheckFunctionExists)

# Find a ros package. 
macro(rosbuild_find_ros_package pkgname) 
  # catch the error output to suppress it 
  execute_process( 
    COMMAND rospack find ${pkgname} 
    ERROR_VARIABLE __rospack_err_ignore 
    OUTPUT_VARIABLE __pkg_dir 
    OUTPUT_STRIP_TRAILING_WHITESPACE) 
  # todo: catch return code and be smart about it 
  set(${pkgname}_PACKAGE_PATH ${__pkg_dir}) 
endmacro(rosbuild_find_ros_package) 

# Retrieve the current COMPILE_FLAGS for the given target, append the new
# ones, and set the result.
macro(rosbuild_add_compile_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} COMPILE_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        COMPILE_FLAGS "${_flags_str}")
endmacro(rosbuild_add_compile_flags)

# Retrieve the current COMPILE_FLAGS for the given target, remove the given
# ones, and set the result.
macro(rosbuild_remove_compile_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} COMPILE_FLAGS)
  separate_arguments(_flags)
  list(REMOVE_ITEM _flags ${args})

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        COMPILE_FLAGS "${_flags_str}")
endmacro(rosbuild_remove_compile_flags)

# Retrieve the current LINK_FLAGS for the given target, append the new
# ones, and set the result.
macro(rosbuild_add_link_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} LINK_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        LINK_FLAGS "${_flags_str}")
endmacro(rosbuild_add_link_flags)

# Retrieve the current LINK_FLAGS for the given target, remove the given
# ones, and set the result.
macro(rosbuild_remove_link_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} LINK_FLAGS)
  separate_arguments(_flags)
  list(REMOVE_ITEM _flags ${args})

  _rosbuild_list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        LINK_FLAGS "${_flags_str}")
endmacro(rosbuild_remove_link_flags)

macro(rosbuild_invoke_rospack pkgname _prefix _varname)
  # Check that our cached location of rospack is valid.  It can be invalid
  # if rospack has moved since last time we ran, #1154.  If it's invalid,
  # search again.
  if(NOT EXISTS ${ROSPACK_EXE})
    message("Cached location of rospack is invalid; searching for rospack...")
    set(ROSPACK_EXE ROSPACK_EXE-NOTFOUND)
    find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable")
    if (NOT ROSPACK_EXE)
      message(FATAL_ERROR "Couldn't find rospack. Please run 'make' in $ROS_ROOT")
    endif(NOT ROSPACK_EXE)
  endif(NOT EXISTS ${ROSPACK_EXE})
  set(_rospack_invoke_result)
  execute_process(
    COMMAND ${ROSPACK_EXE} ${ARGN} ${pkgname}
    OUTPUT_VARIABLE _rospack_invoke_result
    ERROR_VARIABLE _rospack_err_ignore
    RESULT_VARIABLE _rospack_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if (_rospack_failed)
    #set(_rospack_${_varname} "")
    #set(${_prefix}_${_varname} "" CACHE INTERNAL "")
    message("${_rospack_err_ignore}")
    message("${_rospack_invoke_result}")
    message(FATAL_ERROR "\nFailed to invoke rospack to get compile flags for package '${pkgname}.'  Look above for errors from rospack itself.  Aborting.  Please fix the broken dependency!\n")
  else(_rospack_failed)
    separate_arguments(_rospack_invoke_result)
    set(_rospack_${_varname} ${_rospack_invoke_result})
    set(${_prefix}_${_varname} "${_rospack_invoke_result}" CACHE INTERNAL "")
  endif(_rospack_failed)
endmacro(rosbuild_invoke_rospack)

###############################################################################
# This is the user's main entry point.  A *lot* of work gets done here.  It
# should probably be split up into multiple macros.
macro(rosbuild_init)
  # Infer package name from directory name.
  get_filename_component(_project ${PROJECT_SOURCE_DIR} NAME)
  message("[rosbuild] Building package ${_project}")

  project(${_project})

  # Must call include(rosconfig) after project, because rosconfig uses
  # PROJECT_SOURCE_DIR
  include($ENV{ROS_ROOT}/core/rosbuild/rosconfig.cmake)

  # Check that PYTHONPATH includes roslib
  _rosbuild_check_pythonpath()

  # Check that manifest.xml is valid
  _rosbuild_check_manifest()

  # If we're making a distribution, then we don't need to assemble build
  # flags and such.  More to the point, this step will likely fail, because
  # it can rely on call foo-config for a 3rdparty package foo that was
  # cleaned before getting here.
  if(NOT ROSPACK_MAKEDIST)
  
  # Add ROS_PACKAGE_NAME define
  add_definitions(-DROS_PACKAGE_NAME=\\\"${PROJECT_NAME}\\\")

  # ROS_BUILD_TYPE is set by rosconfig
  if(ROS_BUILD_TYPE STREQUAL "Coverage")
    # "Coverage" is our own little target
    set(CMAKE_BUILD_TYPE "Debug")
    set(ROS_COMPILE_FLAGS "-W -Wall -Wno-unused-parameter -fno-strict-aliasing -fprofile-arcs -ftest-coverage")
    set(ROS_LINK_LIBS "gcov")
  else(ROS_BUILD_TYPE STREQUAL "Coverage")
    set(CMAKE_BUILD_TYPE ${ROS_BUILD_TYPE})
  endif(ROS_BUILD_TYPE STREQUAL "Coverage")

  # Set default output directories
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  # By default, look in the local include dir
  include_directories(${PROJECT_SOURCE_DIR}/include)

  set(_prefix ${PROJECT_NAME})
  set(${_prefix}_INCLUDEDIR "" CACHE INTERNAL "")

  # Get the include dirs
  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} INCLUDE_DIRS cflags-only-I --deps-only)
  #message("${pkgname} include dirs: ${${_prefix}_INCLUDE_DIRS}")
  include_directories(${${_prefix}_INCLUDE_DIRS})

  # Get the other cflags
  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} temp cflags-only-other --deps-only)
  _rosbuild_list_to_string(${_prefix}_CFLAGS_OTHER "${${_prefix}_temp}")
  #message("${pkgname} other cflags: ${${_prefix}_CFLAGS_OTHER}")

  # Get the lib dirs
  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} LIBRARY_DIRS libs-only-L --deps-only)
  #message("${pkgname} library dirs: ${${_prefix}_LIBRARY_DIRS}")
  link_directories(${${_prefix}_LIBRARY_DIRS})

  # Get the libs
  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} LIBRARIES libs-only-l --deps-only)
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE ${_prefix}_LIBRARIES)
  #list(REMOVE_DUPLICATES ${_prefix}_LIBRARIES)
  _rosbuild_list_remove_duplicates("${${_prefix}_LIBRARIES}" _tmplist)
  set(${_prefix}_LIBRARIES ${__tmplist})
  list(REVERSE ${_prefix}_LIBRARIES)

  # Also throw in the libs that we want to link everything against (only
  # use case for this so far is -lgcov when building with code coverage
  # support).
  list(APPEND ${_prefix}_LIBRARIES "${ROS_LINK_LIBS}")

  # Get the other lflags
  rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} temp libs-only-other --deps-only)
  _rosbuild_list_to_string(${_prefix}_LDFLAGS_OTHER "${${_prefix}_temp}")
  #message("${pkgname} other ldflags: ${${_prefix}_LDFLAGS_OTHER}")

  #
  # Catch absolute pathnames to archive libraries and bracket them with
  # linker args necessary to force extraction of the entire archive.
  #
  # The OS X linker doesn't accept the -whole-archive and -no-whole-archive
  # arguments.
  #
  if(NOT APPLE)
    foreach(_lib ${${_prefix}_LIBRARIES})
      if(_lib MATCHES "/[^ ]*\\.a")
        set(_bracket_str "-Wl,-whole-archive ${_lib} -Wl,-no-whole-archive")
        list(APPEND ${_prefix}_LDFLAGS_OTHER "${_bracket_str}")
      endif(_lib MATCHES "/[^ ]*\\.a")
    endforeach(_lib)
  endif(NOT APPLE)

  # Also get the full paths to the manifests for all packages on which 
  # we depend
  rosbuild_invoke_rospack(${PROJECT_NAME} _rospack invoke_result deps-manifests)
  set(ROS_MANIFEST_LIST "${PROJECT_SOURCE_DIR}/manifest.xml ${_rospack_invoke_result}")
  # convert whitespace-separated string to ;-separated list
  separate_arguments(ROS_MANIFEST_LIST)

  # Set up the test targets.  Subsequent calls to rosbuild_add_gtest and
  # friends add targets and dependencies from these targets.
  #
  # The 'tests' target builds the test program
  add_custom_target(tests)
  # The 'test' target runs all but the future tests
  add_custom_target(test)
  # Clean out previous test results before running tests.  Use bash
  # conditional to ignore failures (most often happens when a stale NFS
  # handle lingers in the test results directory), because CMake doesn't
  # seem to be able to do it.
  add_custom_command(TARGET tests
                     PRE_BUILD
                     COMMAND if ! rm -rf $ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}\; then echo "WARNING: failed to remove test-results directory"\; fi)
  # The 'test-future' target runs the future tests
  add_custom_target(test-future)


  # Find rostest.  The variable rostest_path will also be reused in other
  # macros.
  rosbuild_invoke_rospack("" rostest path find rostest)
  add_custom_target(test-results-run)
  add_custom_target(test-results
                    COMMAND ${rostest_path}/bin/rostest-results --nodeps ${_project})
  add_dependencies(test-results test-results-run)

  add_custom_target(gcoverage-run)
  add_custom_target(gcoverage 
                    COMMAND rosgcov_summarize ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/.rosgcov_files)
  add_dependencies(gcoverage gcoverage-run)
  file(REMOVE ${PROJECT_SOURCE_DIR}/.rosgcov_files)
  # This doesn't work for some reason...
  #file(GLOB_RECURSE _old_gcov_files ${CMAKE_SOURCE_DIR} *.gcov)
  #message("_old_gcov_files: ${_old_gcov_files}")
  #file(REMOVE "${_old_gcov_files}")

  # Find roslib; roslib_path will be used later
  rosbuild_invoke_rospack("" roslib path find roslib)

  # Figure out which languages we're building for.  "rospack langs" will
  # return a list of packages that:
  #   - depend directly on roslang
  #   - are not in the env var ROS_LANG_DISABLE
  rosbuild_invoke_rospack("" _roslang LANGS langs)
  separate_arguments(_roslang_LANGS)
  set(genmsg_list "")
  set(gensrv_list "")
  # Create a target for client libs attach their message-generation output
  # to
  add_custom_target(rospack_genmsg)
  add_custom_target(rospack_gensrv)
  
  # ${gendeps_exe} is a convenience variable that roslang cmake rules
  # must reference as a dependency of msg/srv generation
  set(gendeps_exe ${roslib_path}/scripts/gendeps) 
      
  # Iterate over the languages, retrieving any exported cmake fragment from
  # each one.
  set(_cmake_fragments)
  foreach(_l ${_roslang_LANGS})
    # Get the roslang attributes from this package.

    # cmake
    rosbuild_invoke_rospack(${_l} ${_l} CMAKE export --lang=roslang --attrib=cmake)
    if(${_l}_CMAKE)
      foreach(_f ${${_l}_CMAKE})
        list(APPEND _cmake_fragments ${_f})
      endforeach(_f)
    endif(${_l}_CMAKE)
  endforeach(_l)

  # Also collect cmake fragments exported by packages that depend on
  # rosbuild.
  rosbuild_invoke_rospack(rosbuild _rosbuild EXPORTS plugins --attrib=cmake --top=${_project})
  list(LENGTH _rosbuild_EXPORTS _rosbuild_EXPORTS_length)

  # rospack plugins outputs the list as:
  # <package name> <attribute value>
  # Here we remove <package name> in all cases by:
  # 1) Remove the first element of the returned list
  # 2) Search for all instances of <newline><string><semicolon>, replacing them with just a semicolon

  # 1) Remove the first package name if the list has at least one element
  if (${_rosbuild_EXPORTS_length} GREATER 0)
    list(REMOVE_AT _rosbuild_EXPORTS 0)
  endif(${_rosbuild_EXPORTS_length} GREATER 0)

  # 2) Remove the rest of the package names
  string(REGEX REPLACE "\n[^;]*;" ";" _rosbuild_EXPORTS_stripped "${_rosbuild_EXPORTS}")

  set(_rosbuild_EXPORTS "" CACHE INTERNAL "")

  foreach(_f ${_rosbuild_EXPORTS_stripped})
    list(APPEND _cmake_fragments ${_f})
  endforeach(_f)

  # Now include them all
  foreach(_f ${_cmake_fragments})
    if(NOT EXISTS ${_f})
      message(FATAL_ERROR "Cannot include non-existent exported cmake file ${_f}")
    endif(NOT EXISTS ${_f})
    # Include this cmake fragment; presumably it will do /
    # provide something useful.  Only include each file once (a file
    # might be multiply referenced because of package dependencies
    # dependencies).
    if(NOT ${_f}_INCLUDED)
      message("[rosbuild] Including ${_f}")
      include(${_f})
      set(${_f}_INCLUDED Y)
    endif(NOT ${_f}_INCLUDED)
  endforeach(_f)


  #
  # Gather the gtest build flags, for use when building unit tests.  We
  # don't require the user to declare a dependency on gtest.
  #
  rosbuild_invoke_rospack(gtest _gtest PACKAGE_PATH find)
  include_directories(${_gtest_PACKAGE_PATH}/gtest/include)
  link_directories(${_gtest_PACKAGE_PATH}/gtest/lib)
  set(_gtest_LIBRARIES -lgtest)
  set(_gtest_CFLAGS_OTHER "")
  set(_gtest_LDFLAGS_OTHER "-Wl,-rpath,${_gtest_PACKAGE_PATH}/gtest/lib")
  
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE _gtest_LIBRARIES)
  #list(REMOVE_DUPLICATES _gtest_LIBRARIES)
  _rosbuild_list_remove_duplicates(${_gtest_LIBRARIES} _tmplist)
  set(_gtest_LIBRARIES ${_tmplist})
  list(REVERSE _gtest_LIBRARIES)

  #
  # Try to get the SVN URL and revision of the package. 
  # TODO: Support other version control systems (svk, git, etc.)
  #
  execute_process(
    COMMAND svn info ${PROJECT_SOURCE_DIR}
    COMMAND grep Revision
    COMMAND cut -d " " -f 2,2
    OUTPUT_VARIABLE _svn_rev
    ERROR_VARIABLE _svn_error
    RESULT_VARIABLE _svn_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  execute_process(
    COMMAND svn info ${PROJECT_SOURCE_DIR}
    COMMAND grep URL
    COMMAND cut -d " " -f 2,2
    OUTPUT_VARIABLE _svn_url
    ERROR_VARIABLE _svn_error
    RESULT_VARIABLE _svn_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(_svn_failed)
    # No big deal 
  else(_svn_failed)
    # We treat the revision as a string
    set(ROS_PACKAGE_REVISION "${_svn_url}:${_svn_rev}")
    # Stop passing this in, because it causes spurious re-builds after svn
    # updates.
    #add_definitions(-DROS_PACKAGE_REVISION=\\\"${ROS_PACKAGE_REVISION}\\\")
    file(WRITE ${PROJECT_SOURCE_DIR}/.build_version ${ROS_PACKAGE_REVISION}\n)
  endif(_svn_failed)
  endif(NOT ROSPACK_MAKEDIST)
endmacro(rosbuild_init)
###############################################################################

# A wrapper around add_executable(), using info from the rospack
# invocation to set up compiling and linking.
macro(rosbuild_add_executable exe)
  add_executable(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  # The SOURCES property seems to be available only since 2.6.  Yar.
  #get_target_property(_srclist ${exe} SOURCES) 
  set(_srclist ${ARGN})
  foreach(_src ${_srclist}) 
    # Handle the case where the second argument is EXCLUDE_FROM_ALL, not a
    # source file.  Only have to do this because we can't get the SOURCES
    # property.
    if(NOT _src STREQUAL EXCLUDE_FROM_ALL)
      set(_file_name _file_name-NOTFOUND)
      find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
      if(NOT _file_name)
        message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
        set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
      endif(NOT _file_name)
      add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST}) 
    endif(NOT _src STREQUAL EXCLUDE_FROM_ALL)
  endforeach(_src)

  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rosbuild_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rosbuild_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rosbuild_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rosbuild_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")

endmacro(rosbuild_add_executable)

# Wrapper around add_library.  We can build shared static and shared libs, and
# set up compile and link flags for both.
macro(rosbuild_add_library lib)

  # Sanity check; must build at least one kind of library.
  if(NOT ROS_BUILD_STATIC_LIBS AND NOT ROS_BUILD_SHARED_LIBS)
    message(FATAL_ERROR "Neither shared nor static libraries are enabled.  Please set either ROS_BUILD_STATIC_LIBS or ROS_BUILD_SHARED_LIBS to true in your $ROS_ROOT/rosconfig.cmake")
  endif(NOT ROS_BUILD_STATIC_LIBS AND NOT ROS_BUILD_SHARED_LIBS)

  # What are we building?
  if(ROS_BUILD_SHARED_LIBS)
    # If shared libs are being built, they get the default CMake target name
    # No matter what, the libraries get the same name in the end.
    _rosbuild_add_library(${lib} ${lib} SHARED ${ARGN})
  endif(ROS_BUILD_SHARED_LIBS)

  if(ROS_BUILD_STATIC_LIBS)
    # If we're only building static libs, then they get the default CMake
    # target name.
    if(NOT ROS_BUILD_SHARED_LIBS)
      set(static_lib_name "${lib}")
    else(NOT ROS_BUILD_SHARED_LIBS)
      set(static_lib_name "${lib}-static")
    endif(NOT ROS_BUILD_SHARED_LIBS)

    _rosbuild_add_library(${static_lib_name} ${lib} STATIC ${ARGN})
  endif(ROS_BUILD_STATIC_LIBS)

endmacro(rosbuild_add_library)

# Wrapper around add_library for the specific case of building a MODULE,
# which works a little differently on dyld systems (e.g., OS X)
macro(rosbuild_add_library_module lib)
  _rosbuild_add_library(${lib} ${lib} MODULE ${ARGN})
endmacro(rosbuild_add_library_module)

# Explicitly add flags for gtest.  We do this here, instead of using
# manifest dependencies, because there are situations in which it is
# undesirable to link in gtest where's it's not being used.  gtest is
# part of the "core" build that happens during a 'make' in ros, so we can
# assume that's already built.
macro(rosbuild_add_gtest_build_flags exe)
  rosbuild_add_compile_flags(${exe} ${_gtest_CFLAGS_OTHER})
  target_link_libraries(${exe} ${_gtest_LIBRARIES})
  rosbuild_add_link_flags(${exe} ${_gtest_LDFLAGS_OTHER})
  rosbuild_declare_test(${exe})
endmacro(rosbuild_add_gtest_build_flags)

# Declare an executable to be a test harness, which excludes it from the
# all target, and adds a dependency to the tests target.
macro(rosbuild_declare_test exe)
  # We provide a 'tests' target that just builds the tests.
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(tests)
  add_dependencies(tests ${exe})
endmacro(rosbuild_declare_test)

# A helper to create test programs.  It calls rosbuild_add_executable() to
# create the program, and augments a test target that was created in the
# call rospack()
macro(rosbuild_add_gtest exe)
  _rosbuild_add_gtest(${ARGV})
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})

  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test test_${_testname})

  # Register check for test output
  _rosbuild_check_rostest_xml_result(test_${_testname} $ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}/${_testname}.xml)
endmacro(rosbuild_add_gtest)

# A helper to create test programs that are expected to fail for the near
# future.  It calls rosbuild_add_executable() to
# create the program, and augments a test target that was created in the
# call rospack()
macro(rosbuild_add_gtest_future exe)
  _rosbuild_add_gtest(${ARGV})
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})

  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future test_${_testname})
endmacro(rosbuild_add_gtest_future)

# A helper to run rostests. It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rosbuild_add_rostest file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_rostest(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test rostest_${_testname})
  _rosbuild_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rosbuild_add_rostest)

# A helper to run rostests that are expected to fail for the near future. 
# It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rosbuild_add_rostest_future file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_rostest(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future rostest_${_testname})
endmacro(rosbuild_add_rostest_future)

# A helper to run rostests that require a graphical display.
# It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rosbuild_add_rostest_graphical file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_rostest(${file} $ENV{ROS_BUILD_XVFB})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test rostest_${_testname})
  _rosbuild_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rosbuild_add_rostest_graphical)

# A helper to run Python unit tests. It generates a command to run python
# the specified file 
macro(rosbuild_add_pyunit file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_pyunit(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test pyunit_${_testname})
endmacro(rosbuild_add_pyunit)

# A helper to run Python unit tests that are expected to fail for the near
# future. It generates a command to run python
# the specified file 
macro(rosbuild_add_pyunit_future file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_pyunit(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future pyunit_${_testname})
endmacro(rosbuild_add_pyunit_future)

# A helper to run pyunit tests that require a graphical display.
# It generates a command to run python on
# the specified file and makes this target a dependency of test. 
macro(rosbuild_add_pyunit_graphical file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_pyunit(${file} $ENV{ROS_BUILD_XVFB})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test pyunit_${_testname})
endmacro(rosbuild_add_pyunit_graphical)

set(_ROSBUILD_GENERATED_MSG_FILES "")
macro(rosbuild_add_generated_msgs)
  list(APPEND _ROSBUILD_GENERATED_MSG_FILES ${ARGV})
endmacro(rosbuild_add_generated_msgs)

# Return a list of all msg/.msg files
macro(rosbuild_get_msgs msgvar)
  file(GLOB _msg_files RELATIVE "${PROJECT_SOURCE_DIR}/msg" "${PROJECT_SOURCE_DIR}/msg/*.msg")
  set(${msgvar} ${_ROSBUILD_GENERATED_MSG_FILES})
  # Loop over each .msg file, establishing a rule to compile it
  foreach(_msg ${_msg_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.msg, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_msg} MATCHES "^[^\\.].*\\.msg$")
      list(APPEND ${msgvar} ${_msg})
    endif(${_msg} MATCHES "^[^\\.].*\\.msg$")
  endforeach(_msg)
endmacro(rosbuild_get_msgs)

set(_ROSBUILD_GENERATED_SRV_FILES "")
macro(rosbuild_add_generated_srvs)
  list(APPEND _ROSBUILD_GENERATED_SRV_FILES ${ARGV})
endmacro(rosbuild_add_generated_srvs)

# Return a list of all srv/.srv files
macro(rosbuild_get_srvs srvvar)
  file(GLOB _srv_files RELATIVE "${PROJECT_SOURCE_DIR}/srv" "${PROJECT_SOURCE_DIR}/srv/*.srv")
  set(${srvvar} ${_ROSBUILD_GENERATED_SRV_FILES})
  # Loop over each .srv file, establishing a rule to compile it
  foreach(_srv ${_srv_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.srv, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_srv} MATCHES "^[^\\.].*\\.srv$")
      list(APPEND ${srvvar} ${_srv})
    endif(${_srv} MATCHES "^[^\\.].*\\.srv$")
  endforeach(_srv)
endmacro(rosbuild_get_srvs)

# Compute msg/srv depenendency list, with simple caching
macro(rosbuild_gendeps _pkg _msgfile)
  # Did we already compute it?
  if(NOT ${_pkg}_${_msgfile}_GENDEPS_COMPUTED)
    # Call out to the gendeps tool to get full paths to .msg files on
    # which this one depends, for proper dependency tracking
    # ${roslib_path} was determined inside rospack()
    execute_process(
      COMMAND ${roslib_path}/scripts/gendeps ${_input}
      OUTPUT_VARIABLE __other_msgs
      ERROR_VARIABLE __rospack_err_ignore 
      OUTPUT_STRIP_TRAILING_WHITESPACE) 
    # For some reason, the output from gendeps has escaped spaces in it.
    # Converting to a string and then back to a list removes them.
    _rosbuild_list_to_string(${_pkg}_${_msgfile}_GENDEPS "${__other_msgs}")
    separate_arguments(${_pkg}_${_msgfile}_GENDEPS)
    set(${_pkg}_${_msgfile}_GENDEPS_COMPUTED Y)
  endif(NOT ${_pkg}_${_msgfile}_GENDEPS_COMPUTED)
endmacro(rosbuild_gendeps)

# gensrv processes srv/*.srv files into language-specific source files
macro(rosbuild_gensrv)
  # Create dummy target that depends on the autogenerated output from all
  # client libs, which has already been attached to the rospack_gensrv
  # target.
  add_custom_target(rospack_gensrv_real ALL)
  add_dependencies(rospack_gensrv_real rospack_gensrv)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/srv/cpp)
endmacro(rosbuild_gensrv)

# genmsg processes msg/*.msg files into language-specific source files
macro(rosbuild_genmsg)
  # Create dummy target that depends on the autogenerated output from all
  # client libs, which has already been attached to the rospack_genmsg
  # target.
  add_custom_target(rospack_genmsg_real ALL)
  add_dependencies(rospack_genmsg_real rospack_genmsg)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/msg/cpp)
endmacro(rosbuild_genmsg)

macro(rosbuild_add_boost_directories)
  execute_process(COMMAND "rosboost-cfg" "--include_dirs"
                  OUTPUT_VARIABLE BOOST_INCLUDE_DIRS
                  RESULT_VARIABLE _boostcfg_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
                  
  if (_boostcfg_failed)
    message(FATAL_ERROR "rosboost-cfg --include_dirs failed")
  endif(_boostcfg_failed)
  
  execute_process(COMMAND "rosboost-cfg" "--lib_dirs"
                  OUTPUT_VARIABLE BOOST_LIB_DIRS
                  RESULT_VARIABLE _boostcfg_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
                  
  if (_boostcfg_failed)
    message(FATAL_ERROR "rosboost-cfg --lib_dirs failed")
  endif(_boostcfg_failed)
  
  add_definitions(-DBOOST_CB_DISABLE_DEBUG)
  include_directories(${BOOST_INCLUDE_DIRS})
  link_directories(${BOOST_LIB_DIRS})
endmacro(rosbuild_add_boost_directories)

macro(rosbuild_link_boost target)
  set(_libs "")
  set(_first 1)
  foreach(arg ${ARGN})
    if (_first)
      set(_first 0)
      set(_libs "${arg}")
    else(_first)
      set(_libs "${_libs},${arg}")
    endif(_first)
  endforeach(arg)

  execute_process(COMMAND "rosboost-cfg" "--libs" ${_libs}
                  OUTPUT_VARIABLE BOOST_LIBS
                  RESULT_VARIABLE _boostcfg_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  
  if (_boostcfg_failed)
    message(FATAL_ERROR "rosboost-cfg --libs failed")
  endif(_boostcfg_failed)

  separate_arguments(BOOST_LIBS)

  target_link_libraries(${target} ${BOOST_LIBS})
endmacro(rosbuild_link_boost)

# Macro to download data on the tests target
macro(rosbuild_download_test_data _url _filename)
  find_package(Wget REQUIRED)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                     COMMAND cmake -E echo "[rosbuild] Downloading ${_url} to ${_filename}..."
                     COMMAND ${WGET_EXECUTABLE} -q ${_url} -O ${PROJECT_SOURCE_DIR}/${_filename}
                     COMMAND cmake -E echo "[rosbuild] Done."
                     VERBATIM)
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  add_custom_target(${_testname}
                    DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(tests)
  add_dependencies(tests ${_testname})
endmacro(rosbuild_download_test_data)

# Macro to download data on the all target
macro(rosbuild_download_data _url _filename)
 find_package(Wget REQUIRED)
 add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                    COMMAND cmake -E echo "[rosbuild] Downloading ${_url} to ${_filename}..."
                    COMMAND ${WGET_EXECUTABLE} -q ${_url} -O ${PROJECT_SOURCE_DIR}/${_filename}
                    COMMAND cmake -E echo "[rosbuild] Done."
                    VERBATIM)
 # Create a legal target name, in case the target name has slashes in it
 string(REPLACE "/" "_" _testname download_data_${_filename})
 add_custom_target(${_testname} ALL
                   DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
endmacro(rosbuild_download_data)

macro(rosbuild_add_openmp_flags target)
# list of OpenMP flags to check
  set(_rospack_check_openmp_flags
    "-fopenmp" # gcc
    "-openmp" # icc
    "-mp" # SGI & PGI
    "-xopenmp" # Sun
    "-omp" # Tru64
    "-qsmp=omp" # AIX
    )

# backup for a variable we will change
  set(_rospack_openmp_flags_backup ${CMAKE_REQUIRED_FLAGS})

# mark the fact we do not yet know the flag
  set(_rospack_openmp_flag_found FALSE)
  set(_rospack_openmp_flag_value)

# find an OpenMP flag that works
  foreach(_rospack_openmp_test_flag ${_rospack_check_openmp_flags})
    if(NOT _rospack_openmp_flag_found)      
      set(CMAKE_REQUIRED_FLAGS ${_rospack_openmp_test_flag})
      check_function_exists(omp_set_num_threads _rospack_openmp_function_found${_rospack_openmp_test_flag})
	   
      if(_rospack_openmp_function_found${_rospack_openmp_test_flag})
	set(_rospack_openmp_flag_value ${_rospack_openmp_test_flag})
	set(_rospack_openmp_flag_found TRUE)
      endif(_rospack_openmp_function_found${_rospack_openmp_test_flag})
    endif(NOT _rospack_openmp_flag_found)
  endforeach(_rospack_openmp_test_flag ${_rospack_check_openmp_flags})

# restore the CMake variable
  set(CMAKE_REQUIRED_FLAGS ${_rospack_openmp_flags_backup})
  
# add the flags or warn
  if(_rospack_openmp_flag_found)
    rosbuild_add_compile_flags(${target} ${_rospack_openmp_flag_value})
    rosbuild_add_link_flags(${target} ${_rospack_openmp_flag_value})
  else(_rospack_openmp_flag_found)
    message("WARNING: OpenMP compile flag not found")
  endif(_rospack_openmp_flag_found)

endmacro(rosbuild_add_openmp_flags)

macro(rosbuild_make_distribution)
  # Infer stack name from directory name.
  get_filename_component(_project ${PROJECT_SOURCE_DIR} NAME)
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
  include(CPack)
endmacro(rosbuild_make_distribution)


