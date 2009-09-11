cmake_minimum_required(VERSION 2.4)

# Policy settings to prevent warnings on 2.6 but ensure proper operation on
# 2.4.
if(COMMAND cmake_policy)
  # Logical target names must be globally unique.
  cmake_policy(SET CMP0002 OLD)
  # Libraries linked via full path no longer produce linker search paths.
  cmake_policy(SET CMP0003 OLD)
  # Preprocessor definition values are now escaped automatically.
  cmake_policy(SET CMP0005 OLD)
endif(COMMAND cmake_policy)

# Use this package to get add_file_dependencies()
include(AddFileDependencies)
# Used to check if a function exists
include(CheckFunctionExists)

###############################################################################
# First things first: we must have rospack.
find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable")
if (NOT ROSPACK_EXE)
  message(FATAL_ERROR "Couldn't find rospack. Please run 'make' in $ROS_ROOT")
endif(NOT ROSPACK_EXE)
###############################################################################

###############################################################################
# Macro to turn a list into a string (why doesn't CMake have this
# built-in?)
macro(_list_to_string _string _list)
    set(${_string})
    foreach(_item ${_list})
        string(LENGTH "${${_string}}" _len)
        if(${_len} GREATER 0)
          set(${_string} "${${_string}} ${_item}")
        else(${_len} GREATER 0)
          set(${_string} "${_item}")
        endif(${_len} GREATER 0)
    endforeach(_item)
endmacro(_list_to_string)

###############################################################################
# Macro to dequote a string, in order to properly construct a command line.
# There must be an easier way to do this.
macro(_dequote_string _out _in)
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
endmacro(_dequote_string)


# list(FIND) was introduced after cmake 2.4.6, so we write our own
macro(_list_find _list _item _idx)
    set(${_idx} -1)
    list(LENGTH ${_list} _len)
    math(EXPR _total "${_len} - 1")
    foreach(_i RANGE ${_total})
      list(GET ${_list} ${_i} _it)
      if(_it STREQUAL ${_item})
        set(${_idx} ${_i})
      endif(_it STREQUAL ${_item})
    endforeach(_i)
endmacro(_list_find)

# list(REMOVE_DUPLICATES) was introduced in cmake 2.6, so we write our own
macro(_list_remove_duplicates _inlist _outlist)
  foreach(_item ${_inlist})
    #list(FIND ${_outlist} ${_item} _idx)
    _list_find(${_outlist} ${_item} _idx)
    if(${_idx} EQUAL -1)
      list(APPEND ${_outlist} ${_item})
    endif(${_idx} EQUAL -1)
  endforeach(_item)
endmacro(_list_remove_duplicates)

# Find a ros package. 
macro(find_ros_package pkgname) 
  # catch the error output to suppress it 
  execute_process( 
    COMMAND rospack find ${pkgname} 
    ERROR_VARIABLE __rospack_err_ignore 
    OUTPUT_VARIABLE __pkg_dir 
    OUTPUT_STRIP_TRAILING_WHITESPACE) 
  # todo: catch return code and be smart about it 
  set(${pkgname}_PACKAGE_PATH ${__pkg_dir}) 
endmacro(find_ros_package) 

# Check validity of PYTHONPATH, to avoid esoteric build errors, #954.
macro(check_pythonpath)
  if("$ENV{PYTHONPATH}" STREQUAL "")
    message("WARNING: PYTHONPATH is not set.  This is almost certainly wrong. Check the ROS installation instructions for details on setting PYTHONPATH.")
  else("$ENV{PYTHONPATH}" STREQUAL "")
    if(NOT $ENV{PYTHONPATH} MATCHES ".*roslib.*")
      message("WARNING: PYTHONPATH does not appear to contain roslib.  This is almost certainly wrong. Check the ROS installation instructions for details on setting PYTHONPATH.")
    endif(NOT $ENV{PYTHONPATH} MATCHES ".*roslib.*")
  endif("$ENV{PYTHONPATH}" STREQUAL "")
endmacro(check_pythonpath)

# Retrieve the current COMPILE_FLAGS for the given target, append the new
# ones, and set the result.
macro(rospack_add_compile_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} COMPILE_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        COMPILE_FLAGS "${_flags_str}")
endmacro(rospack_add_compile_flags)

# Retrieve the current COMPILE_FLAGS for the given target, remove the given
# ones, and set the result.
macro(rospack_remove_compile_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} COMPILE_FLAGS)
  separate_arguments(_flags)
  list(REMOVE_ITEM _flags ${args})

  _list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        COMPILE_FLAGS "${_flags_str}")
endmacro(rospack_remove_compile_flags)

# Retrieve the current LINK_FLAGS for the given target, append the new
# ones, and set the result.
macro(rospack_add_link_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} LINK_FLAGS)
  if(NOT _flags)
    set(_flags ${ARGN})
  else(NOT _flags)
    separate_arguments(_flags)
    list(APPEND _flags "${args}")
  endif(NOT _flags)

  _list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        LINK_FLAGS "${_flags_str}")
endmacro(rospack_add_link_flags)

# Retrieve the current LINK_FLAGS for the given target, remove the given
# ones, and set the result.
macro(rospack_remove_link_flags target)
  set(args ${ARGN})
  separate_arguments(args)
  get_target_property(_flags ${target} LINK_FLAGS)
  separate_arguments(_flags)
  list(REMOVE_ITEM _flags ${args})

  _list_to_string(_flags_str "${_flags}")
  set_target_properties(${target} PROPERTIES
                        LINK_FLAGS "${_flags_str}")
endmacro(rospack_remove_link_flags)

macro(_rospack_invoke pkgname _prefix _varname)
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
endmacro(_rospack_invoke)

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

###############################################################################
# This is the user's main entry point.  A *lot* of work gets done here.  It
# should probably be split up into multiple macros.
macro(rospack _project)

  project(${_project})

  # Must call include(rosconfig) after project, because rosconfig uses
  # PROJECT_SOURCE_DIR
  include($ENV{ROS_ROOT}/core/rosbuild/rosconfig.cmake)

  # Check that PYTHONPATH includes roslib
  check_pythonpath()

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
  _rospack_invoke(${PROJECT_NAME} ${_prefix} INCLUDE_DIRS cflags-only-I --deps-only)
  #message("${pkgname} include dirs: ${${_prefix}_INCLUDE_DIRS}")
  include_directories(${${_prefix}_INCLUDE_DIRS})

  # Get the other cflags
  _rospack_invoke(${PROJECT_NAME} ${_prefix} temp cflags-only-other --deps-only)
  _list_to_string(${_prefix}_CFLAGS_OTHER "${${_prefix}_temp}")
  #message("${pkgname} other cflags: ${${_prefix}_CFLAGS_OTHER}")

  # Get the lib dirs
  _rospack_invoke(${PROJECT_NAME} ${_prefix} LIBRARY_DIRS libs-only-L --deps-only)
  #message("${pkgname} library dirs: ${${_prefix}_LIBRARY_DIRS}")
  link_directories(${${_prefix}_LIBRARY_DIRS})

  # Get the libs
  _rospack_invoke(${PROJECT_NAME} ${_prefix} LIBRARIES libs-only-l --deps-only)
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE ${_prefix}_LIBRARIES)
  #list(REMOVE_DUPLICATES ${_prefix}_LIBRARIES)
  _list_remove_duplicates("${${_prefix}_LIBRARIES}" _tmplist)
  set(${_prefix}_LIBRARIES ${__tmplist})
  list(REVERSE ${_prefix}_LIBRARIES)

  # Also throw in the libs that we want to link everything against (only
  # use case for this so far is -lgcov when building with code coverage
  # support).
  list(APPEND ${_prefix}_LIBRARIES "${ROS_LINK_LIBS}")

  # Get the other lflags
  _rospack_invoke(${PROJECT_NAME} ${_prefix} temp libs-only-other --deps-only)
  _list_to_string(${_prefix}_LDFLAGS_OTHER "${${_prefix}_temp}")
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
  _rospack_invoke(${PROJECT_NAME} _rospack invoke_result deps-manifests)
  set(ROS_MANIFEST_LIST "${PROJECT_SOURCE_DIR}/manifest.xml ${_rospack_invoke_result}")
  # convert whitespace-separated string to ;-separated list
  separate_arguments(ROS_MANIFEST_LIST)

  # Set up the test targets.  Subsequent calls to rospack_add_gtest and
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
  _rospack_invoke("" rostest path find rostest)
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
  _rospack_invoke("" roslib path find roslib)

  # Figure out which languages we're building for.
  _rospack_invoke("" _roslang LANGS langs)
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
      
  foreach(_l ${_roslang_LANGS})
    # Get the roslang attributes from this package.

    # cmake
    _rospack_invoke(${_l} ${_l} CMAKE export --lang=roslang --attrib=cmake)
    if(${_l}_CMAKE)
      foreach(_f ${${_l}_CMAKE})
        if(NOT EXISTS ${_f})
          message(FATAL_ERROR "Package ${_l} exports non-existent cmake file ${_f}")
        endif(NOT EXISTS ${_f})
        # Include this package cmake fragment; presumably it will do /
        # provide something useful.  Only include each file once (a file
        # might be multiply referenced because of inter-client-lib
        # dependencies).
        if(NOT ${_f}_INCLUDED)
          message("[rosbuild] Including ${_f}")
          include(${_f})
          set(${_f}_INCLUDED Y)
        endif(NOT ${_f}_INCLUDED)
      endforeach(_f)
    endif(${_l}_CMAKE)
  endforeach(_l)

  #
  # Gather the gtest build flags, for use when building unit tests.  We
  # don't require the user to declare a dependency on gtest.
  #
  _rospack_invoke(gtest _gtest INCLUDE_DIRS cflags-only-I)
  include_directories(${_gtest_INCLUDE_DIRS})
  _rospack_invoke(gtest _gtest LIBRARY_DIRS libs-only-L)
  link_directories(${_gtest_LIBRARY_DIRS})
  _rospack_invoke(gtest _gtest temp cflags-only-other)
  _list_to_string(_gtest_CFLAGS_OTHER "${_gtest_temp}")
  _rospack_invoke(gtest _gtest LIBRARIES libs-only-l)
  _rospack_invoke(gtest _gtest temp libs-only-other)
  _list_to_string(_gtest_LDFLAGS_OTHER "${_gtest_temp}")
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE _gtest_LIBRARIES)
  #list(REMOVE_DUPLICATES _gtest_LIBRARIES)
  _list_remove_duplicates(${_gtest_LIBRARIES} _tmplist)
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
endmacro(rospack)
###############################################################################

macro(_rospack_add_gcov src exe)
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
endmacro(_rospack_add_gcov)

# A wrapper around add_executable(), using info from the rospack
# invocation to set up compiling and linking.
macro(rospack_add_executable exe)
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
      add_file_dependencies(${CMAKE_CURRENT_SOURCE_DIR}/${_src} ${ROS_MANIFEST_LIST}) 
    endif(NOT _src STREQUAL EXCLUDE_FROM_ALL)
  endforeach(_src)

  rospack_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rospack_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rospack_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rospack_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rospack_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rospack_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")

endmacro(rospack_add_executable)

# Wrapper around add_library.  We can build shared static and shared libs, and
# set up compile and link flags for both.
macro(rospack_add_library lib)

  # Sanity check; must build at least one kind of library.
  if(NOT ROS_BUILD_STATIC_LIBS AND NOT ROS_BUILD_SHARED_LIBS)
    message(FATAL_ERROR "Neither shared nor static libraries are enabled.  Please set either ROS_BUILD_STATIC_LIBS or ROS_BUILD_SHARED_LIBS to true in your $ROS_ROOT/rosconfig.cmake")
  endif(NOT ROS_BUILD_STATIC_LIBS AND NOT ROS_BUILD_SHARED_LIBS)

  # What are we building?
  if(ROS_BUILD_SHARED_LIBS)
    # If shared libs are being built, they get the default CMake target name
    # No matter what, the libraries get the same name in the end.

    # Setup for shared lib build
    add_library(${lib} SHARED ${ARGN})

    # Add explicit dependency of each file on our manifest.xml and those of
    # our dependencies
    # The SOURCES property seems to be available only since 2.6.  Yar.
    #get_target_property(_srclist ${lib} SOURCES)
    set(_srclist ${ARGN})
    foreach(_src ${_srclist})
      add_file_dependencies(${CMAKE_CURRENT_SOURCE_DIR}/${_src} ${ROS_MANIFEST_LIST})
      # Set up for gcov
      _rospack_add_gcov(${_src} ${lib})
    endforeach(_src)

    # Prevent deletion of existing lib of same name
    set_target_properties(${lib} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    # Attach compile and link flags
    rospack_add_compile_flags(${lib} ${${PROJECT_NAME}_CFLAGS_OTHER})
    rospack_add_link_flags(${lib} ${${PROJECT_NAME}_LDFLAGS_OTHER})
    # Link lib against dependent libs
    target_link_libraries(${lib} ${${PROJECT_NAME}_LIBRARIES})

    # Add ROS-wide compile and link flags (usually things like -Wall).  These
    # are set in rosconfig.cmake.
    rospack_add_compile_flags(${lib} ${ROS_COMPILE_FLAGS})
    rospack_add_link_flags(${lib} ${ROS_LINK_FLAGS})

    # Make sure that any messages get generated prior to build this target
    add_dependencies(${lib} rospack_genmsg)
    add_dependencies(${lib} rospack_gensrv)

  endif(ROS_BUILD_SHARED_LIBS)

  if(ROS_BUILD_STATIC_LIBS)
    # If we're only building static libs, then they get the default CMake
    # target name.
    if(NOT ROS_BUILD_SHARED_LIBS)
      set(static_lib_name "${lib}")
    else(NOT ROS_BUILD_SHARED_LIBS)
      set(static_lib_name "${lib}-static")
    endif(NOT ROS_BUILD_SHARED_LIBS)

    # Setup for static lib build
    add_library(${static_lib_name} STATIC ${ARGN})

    # Add explicit dependency of each file on our manifest.xml and those of
    # our dependencies
    # The SOURCES property seems to be available only since 2.6.  Yar.
    #get_target_property(_srclist ${static_lib_name} SOURCES)
    set(_srclist ${ARGN})
    foreach(_src ${_srclist})
      add_file_dependencies(${_src} ${ROS_MANIFEST_LIST})
      # Set up for gcov
      _rospack_add_gcov(${_src} ${static_lib_name})
    endforeach(_src)

    # Set output name to be the same as shared lib (may not work on Windows)
    set_target_properties(${static_lib_name} PROPERTIES OUTPUT_NAME ${lib})
    # Prevent deletion of existing lib of same name
    set_target_properties(${static_lib_name} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
    # Attach compile and link flags
    rospack_add_compile_flags(${static_lib_name} ${${PROJECT_NAME}_CFLAGS_OTHER})
    rospack_add_link_flags(${static_lib_name} ${${PROJECT_NAME}_LDFLAGS_OTHER})
    # Link lib against dependent libs
    target_link_libraries(${static_lib_name} ${${PROJECT_NAME}_LIBRARIES})

    # Add ROS-wide compile and link flags (usually things like -Wall).  These
    # are set in rosconfig.cmake.
    rospack_add_compile_flags(${static_lib_name} ${ROS_COMPILE_FLAGS})
    rospack_add_link_flags(${static_lib_name} ${ROS_LINK_FLAGS})

    # Also add -fPIC, because CMake leaves it out when building static
    # libs, even though it's necessary on 64-bit machines for linking this
    # lib against shared libs downstream.
    rospack_add_compile_flags(${static_lib_name} -fPIC)

    # Make sure that any messages get generated prior to build this target
    add_dependencies(${static_lib_name} rospack_genmsg)
    add_dependencies(${static_lib_name} rospack_gensrv)

  endif(ROS_BUILD_STATIC_LIBS)

endmacro(rospack_add_library)

# Explicitly add flags for gtest.  We do this here, instead of using
# manifest dependencies, because there are situations in which it is
# undesirable to link in gtest where's it's not being used.  gtest is
# part of the "core" build that happens during a 'make' in ros, so we can
# assume that's already built.
macro(rospack_add_gtest_build_flags exe)
  rospack_add_compile_flags(${exe} ${_gtest_CFLAGS_OTHER})
  target_link_libraries(${exe} ${_gtest_LIBRARIES})
  rospack_add_link_flags(${exe} ${_gtest_LDFLAGS_OTHER})
  rospack_declare_test(${exe})
endmacro(rospack_add_gtest_build_flags)

# Declare an executable to be a test harness, which excludes it from the
# all target, and adds a dependency to the tests target.
macro(rospack_declare_test exe)
  # We provide a 'tests' target that just builds the tests.
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(tests)
  add_dependencies(tests ${exe})
endmacro(rospack_declare_test)

# A helper to create test programs.  It calls rospack_add_executable() to
# create the program, and augments a test target that was created in the
# call rospack()
macro(rospack_add_gtest exe)
  rospack_add_gtest_internal(${ARGV})
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})

  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test test_${_testname})

  # Register check for test output
  _rostest_check_xml_result(test_${_testname} $ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}/${_testname}.xml)
endmacro(rospack_add_gtest)

# A helper to create test programs that are expected to fail for the near
# future.  It calls rospack_add_executable() to
# create the program, and augments a test target that was created in the
# call rospack()
macro(rospack_add_gtest_future exe)
  rospack_add_gtest_internal(${ARGV})
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})

  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future test_${_testname})
endmacro(rospack_add_gtest_future)

# helper function to register check that results were generated (#580)
macro(_rostest_check_xml_result test_name test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${rostest_path}/bin/rostest-check-results ${test_file}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-results-run)
  add_dependencies(test-results-run ${test_name}_result)	 
endmacro(_rostest_check_xml_result test_name)

macro(rospack_add_gtest_internal exe)

  # Create the program, with basic + gtest build flags
  rospack_add_executable(${exe} EXCLUDE_FROM_ALL ${ARGN})
  rospack_add_gtest_build_flags(${exe})

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
  # For now, we call _rostest_check_xml_result() in rospack_add_gtest() instead.
  #_rostest_check_xml_result(test_${_testname} $ENV{ROS_ROOT}/test/test_results/${PROJECT_NAME}/${_testname}.xml)

  # Make sure that any messages get generated prior to building this target
  add_dependencies(${exe} rospack_genmsg)
  add_dependencies(${exe} rospack_gensrv)

  # Make sure all test programs are built before running this test
  add_dependencies(test_${_testname} tests)

endmacro(rospack_add_gtest_internal)

# A helper to run rostests. It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rospack_add_rostest file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_rostest_internal(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test rostest_${_testname})
  _rostest_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rospack_add_rostest)

# A helper to run rostests that are expected to fail for the near future. 
# It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rospack_add_rostest_future file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_rostest_internal(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future rostest_${_testname})
endmacro(rospack_add_rostest_future)

# A helper to run rostests that require a graphical display.
# It generates a command to run rostest on
# the specified file and makes this target a dependency of test. 
macro(rospack_add_rostest_graphical file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_rostest_internal(${file} $ENV{ROS_BUILD_XVFB})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test rostest_${_testname})
  _rostest_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rospack_add_rostest_graphical)

# helper function to register check that results were generated (#580)
# this one specifically targets rostest. rostest requires different
# arguments as cmake doesn't know the name of the output file
macro(_rostest_check_rostest_result test_name test_pkg test_file)
  add_custom_target(${test_name}_result
                    COMMAND ${rostest_path}/bin/rostest-check-results --rostest ${test_pkg} ${test_file}
		    VERBATIM)
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-results-run)
  add_dependencies(test-results-run ${test_name}_result)	 
endmacro(_rostest_check_rostest_result test_name)

macro(rospack_add_rostest_internal file)

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
  # For now, we call _rostest_check_xml_result() in rospack_add_rostest() 
  # and rospack_add_rostest_future() instead.
  #_rostest_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rospack_add_rostest_internal)

# A helper to run Python unit tests. It generates a command to run python
# the specified file 
macro(rospack_add_pyunit file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_pyunit_internal(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test pyunit_${_testname})
endmacro(rospack_add_pyunit)

# A helper to run Python unit tests that are expected to fail for the near
# future. It generates a command to run python
# the specified file 
macro(rospack_add_pyunit_future file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_pyunit_internal(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test-future)
  add_dependencies(test-future pyunit_${_testname})
endmacro(rospack_add_pyunit_future)

# A helper to run pyunit tests that require a graphical display.
# It generates a command to run python on
# the specified file and makes this target a dependency of test. 
macro(rospack_add_pyunit_graphical file)
  string(REPLACE "/" "_" _testname ${file})
  rospack_add_pyunit_internal(${file} $ENV{ROS_BUILD_XVFB})
  # Redeclaration of target is to workaround bug in 2.4.6
  add_custom_target(test)
  add_dependencies(test pyunit_${_testname})
endmacro(rospack_add_pyunit_graphical)

macro(rospack_add_pyunit_internal file)

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

endmacro(rospack_add_pyunit_internal)

# Return a list of all msg/.msg files
macro(get_msgs msgvar)
  file(GLOB _msg_files RELATIVE "${PROJECT_SOURCE_DIR}/msg" "${PROJECT_SOURCE_DIR}/msg/*.msg")
  set(${msgvar} "")
  # Loop over each .msg file, establishing a rule to compile it
  foreach(_msg ${_msg_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.msg, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_msg} MATCHES "^[^\\.].*\\.msg$")
      list(APPEND ${msgvar} ${_msg})
    endif(${_msg} MATCHES "^[^\\.].*\\.msg$")
  endforeach(_msg)
endmacro(get_msgs)

# Return a list of all srv/.srv files
macro(get_srvs srvvar)
  file(GLOB _srv_files RELATIVE "${PROJECT_SOURCE_DIR}/srv" "${PROJECT_SOURCE_DIR}/srv/*.srv")
  set(${srvvar} "")
  # Loop over each .srv file, establishing a rule to compile it
  foreach(_srv ${_srv_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.srv, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_srv} MATCHES "^[^\\.].*\\.srv$")
      list(APPEND ${srvvar} ${_srv})
    endif(${_srv} MATCHES "^[^\\.].*\\.srv$")
  endforeach(_srv)
endmacro(get_srvs)

# Compute msg/srv depenendency list, with simple caching
macro(gendeps _pkg _msgfile)
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
    _list_to_string(${_pkg}_${_msgfile}_GENDEPS "${__other_msgs}")
    separate_arguments(${_pkg}_${_msgfile}_GENDEPS)
    set(${_pkg}_${_msgfile}_GENDEPS_COMPUTED Y)
  endif(NOT ${_pkg}_${_msgfile}_GENDEPS_COMPUTED)
endmacro(gendeps)

# gensrv processes srv/*.srv files into language-specific source files
macro(gensrv)
  # Create dummy target that depends on the autogenerated output from all
  # client libs, which has already been attached to the rospack_gensrv
  # target.
  add_custom_target(rospack_gensrv_real ALL)
  add_dependencies(rospack_gensrv_real rospack_gensrv)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/srv/cpp)
endmacro(gensrv)

# genmsg processes msg/*.msg files into language-specific source files
macro(genmsg)
  # Create dummy target that depends on the autogenerated output from all
  # client libs, which has already been attached to the rospack_genmsg
  # target.
  add_custom_target(rospack_genmsg_real ALL)
  add_dependencies(rospack_genmsg_real rospack_genmsg)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/msg/cpp)
endmacro(genmsg)

macro(rospack_add_boost_directories)
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
  
  include_directories(${BOOST_INCLUDE_DIRS})
  link_directories(${BOOST_LIB_DIRS})
endmacro(rospack_add_boost_directories)

macro(rospack_link_boost target)
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
endmacro(rospack_link_boost)

macro(rospack_wget_and_build tarball tarball_url tarball_dir unpack_cmd configure_cmd make_cmd install_cmd)
  find_package(Wget REQUIRED)

  _dequote_string(_unpack_cmd ${unpack_cmd})
  _dequote_string(_configure_cmd ${configure_cmd})
  _dequote_string(_make_cmd ${make_cmd})
  _dequote_string(_install_cmd ${install_cmd})

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
endmacro(rospack_wget_and_build)

macro(rospack_download_test_data _url _filename)
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
endmacro(rospack_download_test_data)



###############################################################################
# NDDL Build Macros for TREX
###############################################################################

#Set the prefix
macro(setup_nddl)
    if(NOT ROSPACK_NDDL_PATH)
        _rospack_invoke(${PROJECT_NAME} ${_prefix} ROSPACK_NDDL_PATH export --lang=nddl --attrib=iflags)
        set(ROSPACK_NDDL_PATH ${${_prefix}_ROSPACK_NDDL_PATH})
    endif(NOT ROSPACK_NDDL_PATH)
    if(NOT NDDL_FILES_CREATED)
        set(NDDL_FILES_CREATED "YES")
        add_custom_target(NDDL_FILES ALL)
    endif(NOT NDDL_FILES_CREATED)
endmacro(setup_nddl)

#Adds an nddl directory. You should probably look at using the manifest if you are doing this.
macro(rospack_add_nddl_directory dir)
    if("${filein}" MATCHES "[ ]^/")
        set(ROSPACK_NDDL_PATH -I${dir} " " ${ROSPACK_NDDL_PATH})
    else("${filein}" MATCHES "[ ]^/")
        set(ROSPACK_NDDL_PATH -I${CMAKE_CURRENT_SOURCE_DIR}/${dir} " " ${ROSPACK_NDDL_PATH})
    endif("${filein}" MATCHES "[ ]^/")
endmacro(rospack_add_nddl_directory)


#Get the dependencies of a nddl file.
macro(nddl_depends _file)
    find_ros_package(trex_ros)

    #Execute the script to get dependencies.
    execute_process(
        COMMAND ${trex_ros_PACKAGE_PATH}/bin/nddl_depends.py "-S${CMAKE_CURRENT_SOURCE_DIR}" " " ${_file} " " ${ROSPACK_NDDL_PATH}
	ERROR_VARIABLE _nddl_depends_error
        RESULT_VARIABLE _nddl_depends_failed
	OUTPUT_VARIABLE nddl_files
	OUTPUT_STRIP_TRAILING_WHITESPACE)

    #If there is and error, complain to the user.
    if(_nddl_depends_failed)
       message("ERROR in nddl_depends:")
       message("STDOUT:\n" ${nddl_files})
       message("STDERR:\n" ${_nddl_depends_error})
       message(ARGS: ${trex_ros_PACKAGE_PATH}/bin/nddl_depends.py " " ${_file} " " ${ROSPACK_NDDL_PATH})
       message("end")
       message(FATAL_ERROR "Failed to get NDDL deps for ${_file}")
    endif(_nddl_depends_failed)
    #message(${_file} "\n" ${nddl_files} "\n")
endmacro(nddl_depends)




#Add an nddl file to the build
macro(rospack_add_nddl _file)

    #Get the dependecies of the NDDL
    nddl_depends(${_file})

    #Get the NDDL compilier's package.
    find_ros_package(trex)
    
    #Write nddl cfg.
    string(REPLACE "-I" ";" _nddl_file_path ${ROSPACK_NDDL_PATH})
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/temp_nddl_gen.cfg 
    	       "<configuration>\n"
	       " <binding nddl=\"Object\" cpp=\"Object\"/>\n"
	       " <binding nddl=\"Timeline\" cpp=\"Timeline\"/>\n"
  	       " <include path=\"${_nddl_file_path}\"/>\n"
	       "</configuration>\n")


    #Convert to a list for cmake
    string(REPLACE "\n" ";" _nddl_deps ${nddl_files})

    #Name the XML
    string(REPLACE ".nddl" ".xml" _nddl_dependency_xml ${_file})


    #Run the parser.
    add_custom_command(OUTPUT ${_nddl_dependency_xml}
                       COMMAND java -jar ${trex_PACKAGE_PATH}/PLASMA/build/lib/nddl.jar --NddlParser -C "${CMAKE_CURRENT_SOURCE_DIR}" --config "temp_nddl_gen.cfg" ${_file}
	               WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                       DEPENDS ${_nddl_deps})




    #Create the target.
    string(REPLACE "\n" "" _nddl_dependency_targ ${_file})
    string(REPLACE "/" "_" _nddl_dependency_targ ${_nddl_dependency_targ})
    add_custom_target(${_nddl_dependency_targ}
                      DEPENDS ${_nddl_dependency_xml})
    # Redeclaration of target is to workaround bug in 2.4.6
    add_custom_target(NDDL_FILES ALL)
    add_dependencies(NDDL_FILES ${_nddl_dependency_targ})
endmacro(rospack_add_nddl)


macro(rospack_add_openmp_flags target)
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
    rospack_add_compile_flags(${target} ${_rospack_openmp_flag_value})
    rospack_add_link_flags(${target} ${_rospack_openmp_flag_value})
  else(_rospack_openmp_flag_found)
    message("WARNING: OpenMP compile flag not found")
  endif(_rospack_openmp_flag_found)

endmacro(rospack_add_openmp_flags)

