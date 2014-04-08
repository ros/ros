# Set a flag to indicate that rosbuild_init() has not been called, so that
# we can later catch out-of-order calls to macros that must be called prior
# to rosbuild_init(), related to #1487.
set(ROSBUILD_init_called 0)

# Use this package to get add_file_dependencies()
include(AddFileDependencies)
# Used to check if a function exists
include(CheckFunctionExists)

# look up python interpreter, store in ${PYTHON_EXECUTABLE}, #3763 BSD supprot
find_package(PythonInterp)

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

# Find a ros stack.
macro(rosbuild_find_ros_stack stackname)
  # catch the error output to suppress it
  execute_process(
    COMMAND rosstack find ${stackname}
    ERROR_VARIABLE __rospack_err_ignore
    OUTPUT_VARIABLE __stack_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  # todo: catch return code and be smart about it
  set(${stackname}_STACK_PATH ${__stack_dir})
endmacro(rosbuild_find_ros_stack)

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
    # Only look in PATH for rospack, #3831
    find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
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
    message("Failed to invoke ${ROSPACK_EXE} ${ARGN} ${pkgname}")
    message("${_rospack_err_ignore}")
    message("${_rospack_invoke_result}")
    message(FATAL_ERROR "\nFailed to invoke rospack to get compile flags for package '${pkgname}'.  Look above for errors from rospack itself.  Aborting.  Please fix the broken dependency!\n")
  else(_rospack_failed)
    separate_arguments(_rospack_invoke_result)
    set(_rospack_${_varname} ${_rospack_invoke_result})
    # We don't cache results that contain newlines, because
    # they make CMake's cache unhappy. This check should only affect calls
    # to `rospack plugins`, which we don't need to cache.
    if(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}")
    else(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}" CACHE INTERNAL "")
    endif(_rospack_invoke_result MATCHES ".*\n.*")
  endif(_rospack_failed)
endmacro(rosbuild_invoke_rospack)

###############################################################################
# This is the user's main entry point.  A *lot* of work gets done here.  It
# should probably be split up into multiple macros.
macro(rosbuild_init)
  # Record that we've been called
  set(ROSBUILD_init_called 1)

  # Don't override the project name if the user said not to, #3119
  if(NOT DEFINED ROSBUILD_DONT_REDEFINE_PROJECT)
    # Infer package name from directory name.
    get_filename_component(_project ${CMAKE_SOURCE_DIR} NAME)
    project(${_project})
  else(NOT DEFINED ROSBUILD_DONT_REDEFINE_PROJECT)
    set(_project ${PROJECT_NAME})
  endif(NOT DEFINED ROSBUILD_DONT_REDEFINE_PROJECT)

  # Declare that the minimum version of OSX we support is 10.6, #3491.
  # For background on how CMAKE_OSX_DEPLOYMENT_TARGET works, see:
  #   http://public.kitware.com/Bug/view.php?id=6195
  if(APPLE)
    SET(CMAKE_OSX_DEPLOYMENT_TARGET "10.6" CACHE STRING "Deployment target for OSX" FORCE)
  endif(APPLE)

  message("[rosbuild] Building package ${_project}")

  # Must call include(rosconfig) after project, because rosconfig uses
  # PROJECT_SOURCE_DIR
  include($ENV{ROS_ROOT}/core/rosbuild/rosconfig.cmake)

  # Check that manifest.xml is valid
  _rosbuild_check_manifest()

  # Check that the package directory is correct
  _rosbuild_check_package_location()

  # force automatic escaping of preprocessor definitions
  cmake_policy(PUSH)
  cmake_policy(SET CMP0005 NEW)
  # Add ROS_PACKAGE_NAME define
  add_definitions(-DROS_PACKAGE_NAME=\"${PROJECT_NAME}\")
  cmake_policy(POP)

  # ROS_BUILD_TYPE is set by rosconfig
  # RelWithAsserts is our own type, not supported by CMake
  if("${ROS_BUILD_TYPE}" STREQUAL "RelWithAsserts")
    set(CMAKE_BUILD_TYPE "")
    set(ROS_COMPILE_FLAGS "-O3 ${ROS_COMPILE_FLAGS}")
  else("${ROS_BUILD_TYPE}" STREQUAL "RelWithAsserts")
    set(CMAKE_BUILD_TYPE ${ROS_BUILD_TYPE})
  endif("${ROS_BUILD_TYPE}" STREQUAL "RelWithAsserts")

  # Set default output directories
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  # By default, look in the local include dir
  include_directories(${PROJECT_SOURCE_DIR}/include)

  set(_prefix ${PROJECT_NAME})
  set(${_prefix}_INCLUDEDIR "" CACHE INTERNAL "")

  # Get the full paths to the manifests for all packages on which
  # we depend
  rosbuild_invoke_rospack(${PROJECT_NAME} _rospack deps_manifests_invoke_result deps-manifests)
  rosbuild_invoke_rospack(${PROJECT_NAME} _rospack msgsrv_gen_invoke_result deps-msgsrv)
  set(ROS_MANIFEST_LIST "${PROJECT_SOURCE_DIR}/manifest.xml ${_rospack_deps_manifests_invoke_result} ${_rospack_msgsrv_gen_invoke_result}")
  # convert whitespace-separated string to ;-separated list
  separate_arguments(ROS_MANIFEST_LIST)

  # Check the time at which we last cached flags against the latest
  # modification time for all manifests that we depend on.  If our cache
  # time is smaller, then we need to rebuild our cached values by calling
  # out to rospack to get flags.  This is an optimization in the service of
  # speeding up the build, #2109.
  _rosbuild_compare_manifests(_rebuild_cache "${_rosbuild_cached_flag_time}" "${${_prefix}_cached_manifest_list}" "${ROS_MANIFEST_LIST}")
  if(_rebuild_cache)
    # Explicitly unset all cached variables, to avoid possible accumulation
    # across builds, #2389.
    set(${_prefix}_INCLUDE_DIRS "" CACHE INTERNAL "")
    set(${_prefix}_CFLAGS_OTHER "" CACHE INTERNAL "")
    set(${_prefix}_LIBRARY_DIRS "" CACHE INTERNAL "")
    set(${_prefix}_LIBRARIES "" CACHE INTERNAL "")
    set(${_prefix}_LDFLAGS_OTHER "" CACHE INTERNAL "")
    set(${_prefix}_cached_manifest_list "" CACHE INTERNAL "")

    message("[rosbuild] Cached build flags older than manifests; calling rospack to get flags")
    # Get the include dirs
    rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} INCLUDE_DIRS cflags-only-I --deps-only)
    #message("${pkgname} include dirs: ${${_prefix}_INCLUDE_DIRS}")
    set(${_prefix}_INCLUDE_DIRS ${${_prefix}_INCLUDE_DIRS} CACHE INTERNAL "")

    # Get the other cflags
    rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} temp cflags-only-other --deps-only)
    _rosbuild_list_to_string(${_prefix}_CFLAGS_OTHER "${${_prefix}_temp}")
    #message("${pkgname} other cflags: ${${_prefix}_CFLAGS_OTHER}")
    set(${_prefix}_CFLAGS_OTHER ${${_prefix}_CFLAGS_OTHER} CACHE INTERNAL "")

    # Get the lib dirs
    rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} LIBRARY_DIRS libs-only-L --deps-only)
    #message("${pkgname} library dirs: ${${_prefix}_LIBRARY_DIRS}")
    set(${_prefix}_LIBRARY_DIRS ${${_prefix}_LIBRARY_DIRS} CACHE INTERNAL "")

    # Get the libs
    rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} LIBRARIES libs-only-l --deps-only)
    #
    # The following code removes duplicate libraries from the link line,
    # saving only the last one.
    #
    if(${_prefix}_LIBRARIES)
      find_package(catkin REQUIRED)
      set(_${_prefix}_LIBRARIES ${${_prefix}_LIBRARIES})
      set(${_prefix}_LIBRARIES "")
      list(REVERSE _${_prefix}_LIBRARIES)
      list_append_unique(${_prefix}_LIBRARIES ${_${_prefix}_LIBRARIES})
      list(REVERSE ${_prefix}_LIBRARIES)
    endif()

    # Also throw in the libs that we want to link everything against (only
    # use case for this so far is -lgcov when building with code coverage
    # support).
    list(APPEND ${_prefix}_LIBRARIES "${ROS_LINK_LIBS}")
    set(${_prefix}_LIBRARIES ${${_prefix}_LIBRARIES} CACHE INTERNAL "")

    # Get the other lflags
    rosbuild_invoke_rospack(${PROJECT_NAME} ${_prefix} temp libs-only-other --deps-only)
    _rosbuild_list_to_string(${_prefix}_LDFLAGS_OTHER "${${_prefix}_temp}")
    #message("${pkgname} other ldflags: ${${_prefix}_LDFLAGS_OTHER}")
    set(${_prefix}_LDFLAGS_OTHER ${${_prefix}_LDFLAGS_OTHER} CACHE INTERNAL "")

    # Record the time at which we cached those values
    _rosbuild_get_clock(_time)
    set(_rosbuild_cached_flag_time ${_time} CACHE INTERNAL "")
    set(${_prefix}_cached_manifest_list ${ROS_MANIFEST_LIST} CACHE INTERNAL "")
  endif(_rebuild_cache)

  # Use the (possibly cached) values returned by rospack.
  include_directories(${${_prefix}_INCLUDE_DIRS})
  link_directories(${${_prefix}_LIBRARY_DIRS})

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

  # Set up the test targets.  Subsequent calls to rosbuild_add_gtest and
  # friends add targets and dependencies from these targets.
  #

  # find rosunit since ROSUNIT_SCRIPTS_DIR and ROSUNIT_EXE will be used later
  find_package(rosunit REQUIRED)

  # Record where we're going to put test results (#2003)
  execute_process(COMMAND ${ROSUNIT_SCRIPTS_DIR}/test_results_dir.py
                  OUTPUT_VARIABLE rosbuild_test_results_dir
                  RESULT_VARIABLE _test_results_dir_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_test_results_dir_failed)
    message(FATAL_ERROR "Failed to invoke '${ROSUNIT_SCRIPTS_DIR}/test_results_dir.py'")
  endif(_test_results_dir_failed)

  # The 'tests' target builds the test program
  if(NOT TARGET tests)
    add_custom_target(tests)
  endif()
  # The 'test' target runs all but the future tests
  add_custom_target(test)
  # We need to build tests before running them.  Addition of this
  # dependency also ensures that old test results get cleaned prior to a
  # new test run.
  # but not if rosbuild_test_nobuild is set, #3008
  if(NOT rosbuild_test_nobuild)
    add_dependencies(test tests)
  endif(NOT rosbuild_test_nobuild)

  # Clean out previous test results before running tests.  Use bash
  # conditional to ignore failures (most often happens when a stale NFS
  # handle lingers in the test results directory), because CMake doesn't
  # seem to be able to do it.
  add_custom_target(rosbuild_clean-test-results
                    if ! rm -rf ${rosbuild_test_results_dir}/${PROJECT_NAME}\; then echo "\"WARNING: failed to remove test-results directory\"\;" fi)
  # Make the tests target depend on rosbuild_clean-test-results, which will ensure
  # that test results are deleted before we try to build tests, and thus
  # before we try to run tests.
  add_dependencies(tests rosbuild_clean-test-results)
  # The 'test-future' target runs the future tests
  add_custom_target(test-future)


  add_custom_target(test-results-run)
  add_custom_target(test-results
                    COMMAND ${ROSUNIT_SCRIPTS_DIR}/summarize_results.py --nodeps ${_project})
  add_dependencies(test-results test-results-run)
  # Do we want coverage reporting (only matters for Python, because
  # Bullseye already collects everything into a single file).
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    add_custom_target(test-results-coverage
                      COMMAND ${ROSUNIT_SCRIPTS_DIR}/pycoverage_to_html.py
                      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    # Make tests run before collecting coverage results
    add_dependencies(test-results-coverage test-results-run)
    # Make coverage collection happen
    add_dependencies(test-results test-results-coverage)
  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")

  # Initialize a counter to be used to uniquify target names for
  # calls to rosbuild_add_roslaunch_check(), #3674.
  set(ROSBUILD_roslaunch_check_count 1)

  # Find roslib; roslib_path will be used later
  rosbuild_invoke_rospack("" roslib path find roslib)

  # Create targets for client libs attach their message-generation output to
  add_custom_target(rospack_genmsg)
  add_custom_target(rospack_gensrv)

  # Add a target that will fire before doing message or service generation.
  # This is used by packages that do automatic generation of message and
  # service files.
  add_custom_target(rosbuild_premsgsrvgen)

  # Add a target that will fire before compiling anything.  This is used by
  # message and service generation, as well as things outside of ros, like
  # dynamic_reconfigure.
  add_custom_target(rosbuild_precompile)

  # The rospack_genmsg_libexe target is defined for backward compatibility,
  # and will eventually be removed.
  add_custom_target(rospack_genmsg_libexe)
  add_dependencies(rosbuild_precompile rospack_genmsg_libexe)

  # ${gendeps_exe} is a convenience variable that roslang cmake rules
  # must reference as a dependency of msg/srv generation
  # ${gendeps_exe} is set by roslib-extras.cmake
  find_package(catkin REQUIRED COMPONENTS roslib)

  # If the roslang package is available, pull in cmake/roslang.cmake from
  # there; it will in turn include message-generation logic from client
  # libs.  This is to allow roslang to live outside the ros stack, #3108.
  rosbuild_find_ros_package("roslang")
  if(roslang_PACKAGE_PATH)
    # Can't use rosbuild_include() here, because there's no guarantee that
    # the package we're building depends on roslang (in fact, it probably
    # doesn't.
    include("${roslang_PACKAGE_PATH}/cmake/roslang.cmake")
  endif(roslang_PACKAGE_PATH)

  # Collect cmake fragments exported by packages that depend on
  # rosbuild.  This behavior is deprecated, in favor of using
  # rosbuild_include() to explicitly include cmake code from other packages.
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

  set(_cmake_fragments)
  foreach(_f ${_rosbuild_EXPORTS_stripped})
    list(APPEND _cmake_fragments ${_f})
    message("\n[rosbuild] WARNING: the file ${_f} is being included automatically.  This behavior is deprecated.  The package containing that file should instead export the directory containing the file, and you should use rosbuild_include() to include the file explicitly.\n")
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


  # gtest has been found by catkin
  if(GTEST_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
  endif()

  # Delete the files that let rospack know messages/services have been generated
  file(REMOVE ${PROJECT_SOURCE_DIR}/msg_gen/generated)
  file(REMOVE ${PROJECT_SOURCE_DIR}/srv_gen/generated)
endmacro(rosbuild_init)
###############################################################################

# A wrapper around add_executable(), using info from the rospack
# invocation to set up compiling and linking.
macro(rosbuild_add_executable exe)
  add_executable(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  get_target_property(_srclist ${exe} SOURCES)
  foreach(_src ${_srclist})
    set(_file_name _file_name-NOTFOUND)
    find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
    if(NOT _file_name)
      message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
      set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
    endif(NOT _file_name)
    add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST})
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

  # Make sure to do any prebuild work (e.g., msg/srv generation) before
  # building this target.
  add_dependencies(${exe} rosbuild_precompile)

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
  # Sanity check; it's too hard to support building shared libs and static
  # executables.
  if(ROS_BUILD_STATIC_EXES AND ROS_BUILD_SHARED_LIBS)
    message(FATAL_ERROR "Static executables are requested, but so are shared libs. This configuration is unsupported.  Please either set ROS_BUILD_SHARED_LIBS to false or set ROS_BUILD_STATIC_EXES to false.")
  endif(ROS_BUILD_STATIC_EXES AND ROS_BUILD_SHARED_LIBS)

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
  if (NOT GTEST_FOUND)
    message(WARNING "GTest not found; C++ tests will fail to build.")
  endif()
  if (LIBRARY_OUTPUT_PATH)
    # add this as GTest builds there
    set_target_properties(${exe} PROPERTIES LINK_FLAGS "-L${LIBRARY_OUTPUT_PATH}")
  endif()
  target_link_libraries(${exe} ${GTEST_LIBRARIES})
  link_directories(${GTEST_LIBRARY_DIRS})
  rosbuild_declare_test(${exe})
  add_dependencies(${exe} gtest gtest_main)
endmacro(rosbuild_add_gtest_build_flags)

# Declare an executable to be a test harness, which excludes it from the
# all target, and adds a dependency to the tests target.
macro(rosbuild_declare_test exe)
  # We provide a 'tests' target that just builds the tests.
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(tests)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(tests ${exe})
  # Don't build this target with the all target, #3110
  set_target_properties(${exe} PROPERTIES EXCLUDE_FROM_ALL TRUE)
endmacro(rosbuild_declare_test)

# A helper to create test programs.  It calls rosbuild_add_executable() to
# create the program, and augments a test target that was created in the
# call rospack()
macro(rosbuild_add_gtest exe)
  _rosbuild_add_gtest(${ARGV})
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname ${exe})
  # Redeclaration of test target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test test_${_testname})
  # Register check for test output
  _rosbuild_check_rostest_xml_result(${_testname} ${rosbuild_test_results_dir}/${PROJECT_NAME}/rostest-${_testname}.xml)
endmacro(rosbuild_add_gtest)

# A version of add_gtest that checks a label against ROS_BUILD_TEST_LABEL
macro(rosbuild_add_gtest_labeled label)
  if("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
    rosbuild_add_gtest(${ARGN})
  endif("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
endmacro(rosbuild_add_gtest_labeled)

# A helper to run rostests. It generates a command to run rostest on
# the specified file and makes this target a dependency of test.
macro(rosbuild_add_rostest file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_rostest(${file})
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test rostest_${_testname})
  _rosbuild_check_rostest_result(rostest_${_testname} ${PROJECT_NAME} ${file})
endmacro(rosbuild_add_rostest)

# A version of add_rostest that checks a label against ROS_BUILD_TEST_LABEL
macro(rosbuild_add_rostest_labeled label)
  if("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
    rosbuild_add_rostest(${ARGN})
  endif("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
endmacro(rosbuild_add_rostest_labeled)

# A helper to run Python unit tests. It generates a command to run python
# the specified file
macro(rosbuild_add_pyunit file)
  string(REPLACE "/" "_" _testname ${file})
  _rosbuild_add_pyunit(${ARGV})
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test pyunit_${_testname})
  # Register check for test output
  _rosbuild_check_rostest_xml_result(${_testname} ${rosbuild_test_results_dir}/${PROJECT_NAME}/rosunit-${_testname}.xml)
endmacro(rosbuild_add_pyunit)

# A version of add_pyunit that checks a label against ROS_BUILD_TEST_LABEL
macro(rosbuild_add_pyunit_labeled label)
  if("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
    rosbuild_add_pyunit(${ARGN})
  endif("$ENV{ROS_BUILD_TEST_LABEL}" STREQUAL "" OR "${label}" STREQUAL "$ENV{ROS_BUILD_TEST_LABEL}")
endmacro(rosbuild_add_pyunit_labeled)

# Declare as a unit test a check of a roslaunch file, or a directory
# containing roslaunch files.  Following the file/directory, you can
# specify environment variables as var=val var=val ...
macro(rosbuild_add_roslaunch_check file)
  string(REPLACE "/" "_" _testname ${file})
  # Append a unique count to avoid target name collisions when the same
  # launch file is tested in different environment variable configurations,
  # #3674.  The counter is initialized in rosbuild_init().
  set(_targetname roslaunch_check_${_testname}_${ROSBUILD_roslaunch_check_count})
  # Increment counter for the next call
  math(EXPR ROSBUILD_roslaunch_check_count "${ROSBUILD_roslaunch_check_count} + 1")
  _rosbuild_add_roslaunch_check(${_targetname} ${ARGV})
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(test)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(test ${_targetname})
endmacro(rosbuild_add_roslaunch_check)

set(_ROSBUILD_GENERATED_MSG_FILES "")
macro(rosbuild_add_generated_msgs)
  if(ROSBUILD_init_called)
    message(FATAL_ERROR "rosbuild_add_generated_msgs() cannot be called after rosbuild_init()")
  endif(ROSBUILD_init_called)
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
  if(ROSBUILD_init_called)
    message(FATAL_ERROR "rosbuild_add_generated_srvs() cannot be called after rosbuild_init()")
  endif(ROSBUILD_init_called)
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
    execute_process(
      COMMAND ${gendeps_exe} ${_input}
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
  # roslang_PACKAGE_PATH was set in rosbuild_init(), if roslang was found
  if(NOT roslang_PACKAGE_PATH)
    _rosbuild_warn("rosbuild_gensrv() was called, but the roslang package cannot be found. Service generation will NOT occur")
  endif(NOT roslang_PACKAGE_PATH)
  # Check whether there are any .srv files
  rosbuild_get_srvs(_srvlist)
  if(NOT _srvlist)
    _rosbuild_warn("rosbuild_gensrv() was called, but no .srv files were found")
  else(NOT _srvlist)
    file(WRITE ${PROJECT_SOURCE_DIR}/srv_gen/generated "yes")
    # Now set the mtime to something consistent.  We only want whether or not this file exists to matter
    # But we set it to the current time, because setting it to zero causes
    # annoying warning, #3396.
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import os; os.utime('${PROJECT_SOURCE_DIR}/srv_gen/generated', (1, 1))"
      ERROR_VARIABLE _set_mtime_error
      RESULT_VARIABLE _set_mtime_failed
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_set_mtime_failed)
      message("[rosbuild] Error from calling to Python to set the mtime on ${PROJECT_SOURCE_DIR}/srv_gen/generated:")
      message("${_mtime_error}")
      message(FATAL_ERROR "[rosbuild] Failed to set mtime; aborting")
    endif(_set_mtime_failed)
  endif(NOT _srvlist)
  # Create target to trigger service generation in the case where no libs
  # or executables are made.
  add_custom_target(rospack_gensrv_all ALL)
  add_dependencies(rospack_gensrv_all rospack_gensrv)
  # Make the precompile target, on which libraries and executables depend,
  # depend on the message generation.
  add_dependencies(rosbuild_precompile rospack_gensrv)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/srv_gen/cpp/include)
endmacro(rosbuild_gensrv)

# genmsg processes msg/*.msg files into language-specific source files
macro(rosbuild_genmsg)
  # roslang_PACKAGE_PATH was set in rosbuild_init(), if roslang was found
  if(NOT roslang_PACKAGE_PATH)
    _rosbuild_warn("rosbuild_genmsg() was called, but the roslang package cannot be found.  Message generation will NOT occur")
  endif(NOT roslang_PACKAGE_PATH)
  # Check whether there are any .srv files
  rosbuild_get_msgs(_msglist)
  if(NOT _msglist)
    _rosbuild_warn("rosbuild_genmsg() was called, but no .msg files were found")
  else(NOT _msglist)
    file(WRITE ${PROJECT_SOURCE_DIR}/msg_gen/generated "yes")
    # Now set the mtime to something consistent.  We only want whether or not this file exists to matter
    # But we set it to the current time, because setting it to zero causes
    # annoying warning, #3396.
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import os; os.utime('${PROJECT_SOURCE_DIR}/msg_gen/generated', (1, 1))"
      ERROR_VARIABLE _set_mtime_error
      RESULT_VARIABLE _set_mtime_failed
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_set_mtime_failed)
      message("[rosbuild] Error from calling to Python to set the mtime on ${PROJECT_SOURCE_DIR}/msg_gen/generated:")
      message("${_mtime_error}")
      message(FATAL_ERROR "[rosbuild] Failed to set mtime; aborting")
    endif(_set_mtime_failed)
  endif(NOT _msglist)
  # Create target to trigger message generation in the case where no libs
  # or executables are made.
  add_custom_target(rospack_genmsg_all ALL)
  add_dependencies(rospack_genmsg_all rospack_genmsg)
  # Make the precompile target, on which libraries and executables depend,
  # depend on the message generation.
  add_dependencies(rosbuild_precompile rospack_genmsg)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/msg_gen/cpp/include)
endmacro(rosbuild_genmsg)

macro(rosbuild_add_boost_directories)
  set(_sysroot "--sysroot=${CMAKE_FIND_ROOT_PATH}")
  execute_process(COMMAND "rosboost-cfg" ${_sysroot} "--include_dirs"
                  OUTPUT_VARIABLE BOOST_INCLUDE_DIRS
                  RESULT_VARIABLE _boostcfg_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (_boostcfg_failed)
    message(FATAL_ERROR "rosboost-cfg --include_dirs failed")
  endif(_boostcfg_failed)

  set(_sysroot "--sysroot=${CMAKE_FIND_ROOT_PATH}")
  execute_process(COMMAND "rosboost-cfg" ${_sysroot} "--lib_dirs"
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
  set(_sysroot "--sysroot=${CMAKE_FIND_ROOT_PATH}")
  # We ask for --lflags, as recommended in #3773.  This means that we might
  # pass non-library arguments to target_link_libraries() below, but CMake
  # doesn't seem to mind.
  execute_process(COMMAND "rosboost-cfg" ${_sysroot} "--lflags" ${_libs}
                  OUTPUT_VARIABLE BOOST_LIBS
		  ERROR_VARIABLE _boostcfg_error
                  RESULT_VARIABLE _boostcfg_failed
                  OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (_boostcfg_failed)
    message(FATAL_ERROR "[rosboost-cfg --libs ${_libs}] failed with error: ${_boostcfg_error}")
  endif(_boostcfg_failed)

  separate_arguments(BOOST_LIBS)

  target_link_libraries(${target} ${BOOST_LIBS})
endmacro(rosbuild_link_boost)

# Macro to download data on the tests target
# The real signature is:
#macro(rosbuild_download_test_data _url _filename _md5)
macro(rosbuild_download_test_data _url _filename)
  if("${ARGN}" STREQUAL "")
    _rosbuild_warn("The 2-argument rosbuild_download_test_data(url file) is deprecated; please switch to the 3-argument form, supplying an md5sum for the file: rosbuild_download_test_data(url file md5)")
  endif("${ARGN}" STREQUAL "")

  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                     COMMAND $ENV{ROS_ROOT}/core/rosbuild/bin/download_checkmd5.py ${_url} ${PROJECT_SOURCE_DIR}/${_filename} ${ARGN}
                     VERBATIM)
  add_custom_target(${_testname} DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
  # Redeclaration of target is to workaround bug in 2.4.6
  if(CMAKE_MINOR_VERSION LESS 6)
    add_custom_target(tests)
  endif(CMAKE_MINOR_VERSION LESS 6)
  add_dependencies(tests ${_testname})
endmacro(rosbuild_download_test_data)

# There's an optional 3rd arg, which is a target that should be made to
# depend on the result of untarring the file (can be ALL).
macro(rosbuild_untar_file _filename _unpacked_name)
  get_filename_component(unpack_dir ${_filename} PATH)
  # Check whether the filename has a directory component, #3034
  if(NOT unpack_dir)
    set(unpack_dir ${PROJECT_SOURCE_DIR})
  endif(NOT unpack_dir)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_unpacked_name}
                     COMMAND rm -rf ${PROJECT_SOURCE_DIR}/${_unpacked_name}
                     COMMAND tar xvCf ${unpack_dir} ${PROJECT_SOURCE_DIR}/${_filename}
                     COMMAND touch -c ${PROJECT_SOURCE_DIR}/${_unpacked_name}
                     DEPENDS ${PROJECT_SOURCE_DIR}/${_filename}
                     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                     VERBATIM)
  string(REPLACE "/" "_" _target_name untar_file_${_filename}_${_unpacked_name})

  if("${ARGN}" STREQUAL "ALL")
    add_custom_target(${_target_name} ALL DEPENDS ${PROJECT_SOURCE_DIR}/${_unpacked_name})
  else("${ARGN}" STREQUAL "ALL")
    add_custom_target(${_target_name} DEPENDS ${PROJECT_SOURCE_DIR}/${_unpacked_name})
    if(NOT "${ARGN}" STREQUAL "")
      # Redeclaration of target is to workaround bug in 2.4.6
      if(CMAKE_MINOR_VERSION LESS 6)
        add_custom_target(${ARGN})
      endif(CMAKE_MINOR_VERSION LESS 6)
      add_dependencies(${ARGN} ${_target_name})
    endif(NOT "${ARGN}" STREQUAL "")
  endif("${ARGN}" STREQUAL "ALL")
endmacro(rosbuild_untar_file)

# Macro to download data on the all target
# The real signature is:
#macro(rosbuild_download_data _url _filename _md5)
macro(rosbuild_download_data _url _filename)
  if("${ARGN}" STREQUAL "")
    _rosbuild_warn("The 2-argument rosbuild_download_data(url file) is deprecated; please switch to the 3-argument form, supplying an md5sum for the file: rosbuild_download_data(url file md5)")
  endif("${ARGN}" STREQUAL "")
  # Create a legal target name, in case the target name has slashes in it
  string(REPLACE "/" "_" _testname download_data_${_filename})
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/${_filename}
                    COMMAND $ENV{ROS_ROOT}/core/rosbuild/bin/download_checkmd5.py ${_url} ${PROJECT_SOURCE_DIR}/${_filename} ${ARGN}
                    VERBATIM)
  add_custom_target(${_testname} ALL DEPENDS ${PROJECT_SOURCE_DIR}/${_filename})
endmacro(rosbuild_download_data)

macro(rosbuild_add_openmp_flags target)
  # Bullseye's wrappers appear to choke on OpenMP pragmas.  So if
  # ROS_TEST_COVERAGE is set (which indicates that we're doing a coverage
  # build with Bullseye), we make this macro a no-op.
  if("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
    _rosbuild_warn("because ROS_TEST_COVERAGE is set, OpenMP support is disabled")
  else("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")

  # First, try to use the standard FindOpenMP module (#3184).  If that
  # fails, fall back on manual detection.  I don't know why the standard
  # detection would ever fail; it's possible that the manual
  # detection is now vestigial and could be removed.
  include(FindOpenMP)
  if(${OPENMP_FOUND} STREQUAL "TRUE")
      rosbuild_add_compile_flags(${target} ${OpenMP_C_FLAGS})
      rosbuild_add_link_flags(${target} ${OpenMP_C_FLAGS})
  else(${OPENMP_FOUND} STREQUAL "TRUE")
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

  endif(${OPENMP_FOUND} STREQUAL "TRUE")
  endif("$ENV{ROS_TEST_COVERAGE}" STREQUAL "1")
endmacro(rosbuild_add_openmp_flags)

macro(rosbuild_make_distribution)
  # Noop.  This logic is handled in Python instead.
endmacro(rosbuild_make_distribution)

# Compute the number of hardware cores on the machine.  Intended to use for
# gating tests that have heavy processor requirements.
include($ENV{ROS_ROOT}/core/rosbuild/ProcessorCount.cmake)
macro(rosbuild_count_cores num)
  ProcessorCount(${num})
endmacro(rosbuild_count_cores)

# Check whether we're running as a VM Intended to use for
# gating tests that have heavy processor requirements.  It checks for
# /proc/xen
macro(rosbuild_check_for_vm var)
  set(_xen_dir _xen_dir-NOTFOUND)
  find_file(_xen_dir "xen" PATHS "/proc" NO_DEFAULT_PATH)
  if(_xen_dir)
    set(${var} 1)
  else(_xen_dir)
    set(${var} 0)
  endif(_xen_dir)
endmacro(rosbuild_check_for_vm var)

# Check whether there's an X display.  Intended to use in gating tests that
# require a display.
macro(rosbuild_check_for_display var)
  execute_process(COMMAND "xdpyinfo"
                  OUTPUT_VARIABLE _dummy
                  ERROR_VARIABLE _dummy
                  RESULT_VARIABLE _xdpyinfo_failed)
  if(_xdpyinfo_failed)
    set(${var} 0)
  else(_xdpyinfo_failed)
    set(${var} 1)
  endif(_xdpyinfo_failed)
endmacro(rosbuild_check_for_display)

macro(rosbuild_add_swigpy_library target lib)
  rosbuild_add_library(${target} ${ARGN})
  # swig python needs a shared library named _<modulename>.[so|dll|...]
  # this renames the output file to conform to that by prepending
  # an underscore in place of the "lib" prefix.
  # If on Darwin, force the suffix so ".so", because the MacPorts
  # version of Python won't find _foo.dylib for 'import _foo'
  if(APPLE)
    set_target_properties(${target}
                          PROPERTIES OUTPUT_NAME ${lib}
                          PREFIX "_" SUFFIX ".so")
  else(APPLE)
    set_target_properties(${target}
                          PROPERTIES OUTPUT_NAME ${lib}
                          PREFIX "_")
  endif(APPLE)
endmacro(rosbuild_add_swigpy_library)

macro(rosbuild_check_for_sse)
  # check for SSE extensions
  include(CheckCXXSourceRuns)
  if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
   set(SSE_FLAGS)

   set(CMAKE_REQUIRED_FLAGS "-msse3")
   check_cxx_source_runs("
    #include <pmmintrin.h>

    int main()
    {
       __m128d a, b;
       double vals[2] = {0};
       a = _mm_loadu_pd(vals);
       b = _mm_hadd_pd(a,a);
       _mm_storeu_pd(vals, b);
       return 0;
    }"
    HAS_SSE3_EXTENSIONS)

   set(CMAKE_REQUIRED_FLAGS "-msse2")
   check_cxx_source_runs("
    #include <emmintrin.h>

    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)

   set(CMAKE_REQUIRED_FLAGS "-msse")
   check_cxx_source_runs("
    #include <xmmintrin.h>
    int main()
    {
        __m128 a, b;
        float vals[4] = {0};
        a = _mm_loadu_ps(vals);
        b = a;
        b = _mm_add_ps(a,b);
        _mm_storeu_ps(vals,b);
        return 0;
    }"
    HAS_SSE_EXTENSIONS)

   set(CMAKE_REQUIRED_FLAGS)

   if(HAS_SSE3_EXTENSIONS)
    set(SSE_FLAGS "-msse3 -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE3 extensions, using flags: ${SSE_FLAGS}")
   elseif(HAS_SSE2_EXTENSIONS)
    set(SSE_FLAGS "-msse2 -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE2 extensions, using flags: ${SSE_FLAGS}")
   elseif(HAS_SSE_EXTENSIONS)
    set(SSE_FLAGS "-msse -mfpmath=sse")
    message(STATUS "[rosbuild] Found SSE extensions, using flags: ${SSE_FLAGS}")
   endif()
  elseif(MSVC)
   check_cxx_source_runs("
    #include <emmintrin.h>

    int main()
    {
        __m128d a, b;
        double vals[2] = {0};
        a = _mm_loadu_pd(vals);
        b = _mm_add_pd(a,a);
        _mm_storeu_pd(vals,b);
        return 0;
     }"
     HAS_SSE2_EXTENSIONS)
   if( HAS_SSE2_EXTENSIONS )
    message(STATUS "[rosbuild] Found SSE2 extensions")
    set(SSE_FLAGS "/arch:SSE2 /fp:fast -D__SSE__ -D__SSE2__" )
   endif()
  endif()
endmacro(rosbuild_check_for_sse)

macro(rosbuild_include pkg module)
  # Find exported cmake directories
  rosbuild_invoke_rospack(rosbuild _rosbuild EXPORTS plugins --attrib=cmake_directory --top=${_project})
  string(REGEX REPLACE "\n" ";" _rosbuild_EXPORTS_stripped "${_rosbuild_EXPORTS}")
  list(LENGTH _rosbuild_EXPORTS_stripped _rosbuild_EXPORTS_stripped_length)
  set(_idx 0)
  set(_found False)
  while(_idx LESS ${_rosbuild_EXPORTS_stripped_length} AND NOT _found)
    list(GET _rosbuild_EXPORTS_stripped ${_idx} _pkg)
    math(EXPR _idx "${_idx} + 1")
    list(GET _rosbuild_EXPORTS_stripped ${_idx} _dir)
    if("${_pkg}" STREQUAL "${pkg}")
      message("[rosbuild] Including ${_dir}/${module}.cmake")
      # Add this directory to CMake's search path, so that included files
      # can include other files in a relative manner, #2813.
      if(NOT CMAKE_MODULE_PATH MATCHES ${_dir})
        set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${_dir})
      endif()
      include(${_dir}/${module}.cmake)
      # Poor man's break
      set(_found True)
    endif("${_pkg}" STREQUAL "${pkg}")
    math(EXPR _idx "${_idx} + 1")
  endwhile(_idx LESS ${_rosbuild_EXPORTS_stripped_length} AND NOT _found)
  if(NOT _found)
    message(FATAL_ERROR "[rosbuild] Failed to include ${module} from ${pkg}")
  endif(NOT _found)
endmacro(rosbuild_include)

macro(rosbuild_get_package_version _var pkgname)
  execute_process(
    COMMAND rosstack contains ${pkgname}
    ERROR_VARIABLE __rosstack_err_ignore
    OUTPUT_VARIABLE __stack
    RESULT_VARIABLE _rosstack_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_rosstack_failed OR NOT __stack)
    message(FATAL_ERROR "[rosbuild] Failed to find stack containing package ${pkgname}")
  else(_rosstack_failed OR NOT __stack)
    rosbuild_get_stack_version(${_var} ${__stack})
  endif(_rosstack_failed OR NOT __stack)
endmacro(rosbuild_get_package_version)

macro(rosbuild_get_stack_version _var stackname)
  execute_process(
    COMMAND rosversion ${stackname}
    ERROR_VARIABLE __rosversion_err_ignore
    OUTPUT_VARIABLE __version
    RESULT_VARIABLE _rosversion_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_rosversion_failed OR NOT __version)
    message(FATAL_ERROR "[rosbuild] Failed to find version of stack ${stackname}")
  else(_rosversion_failed OR NOT __version)
    set(${_var} ${__version})
  endif(_rosversion_failed OR NOT __version)
endmacro(rosbuild_get_stack_version _var stackname)

# *_future() calls are deprecated
macro(rosbuild_add_gtest_future exe)
  _rosbuild_warn("The *_future() macros are deprecated.")
  # Still need to create the executable, just in case somebody is using
  # this macro, then doing something that refers to the executable target.
  _rosbuild_add_gtest(${ARGV})
endmacro(rosbuild_add_gtest_future)

macro(rosbuild_add_rostest_future file)
  _rosbuild_warn("The *_future() macros are deprecated.")
endmacro(rosbuild_add_rostest_future)

macro(rosbuild_add_pyunit_future file)
  _rosbuild_warn("The *_future() macros are deprecated.")
endmacro(rosbuild_add_pyunit_future)

