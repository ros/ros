cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rospack_add_pyunit(test/utest.py)
rospack_add_gtest(test/utest test/utest.cpp)

add_custom_target(deepdirs
                  COMMAND python deep.py
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/deep
                  DEPENDS test/deep/deep.py
                  VERBATIM)
add_dependencies(pyunit_test_utest.py deepdirs)

