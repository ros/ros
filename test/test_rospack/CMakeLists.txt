if(ROSBUILD)
  # this is for rosbuild2 integration, you can ignore this
  include(rosbuild.cmake OPTIONAL)
  return()
endif()
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_boost_directories()
rosbuild_add_pyunit(test/utest.py TIMEOUT 180.0)
rosbuild_add_pyunit(test/utest_rosstack.py TIMEOUT 180.0)

rosbuild_add_gtest(test/utest test/utest.cpp TIMEOUT 180.0)
rosbuild_link_boost(test/utest thread)

# look up python interpreter, store in ${PYTHON_EXECUTABLE}
find_package(PythonInterp)

add_custom_target(deepdirs
                  COMMAND ${PYTHON_EXECUTABLE} deep.py
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/deep
                  DEPENDS test/deep/deep.py
                  VERBATIM)
add_dependencies(pyunit_test_utest.py deepdirs)

