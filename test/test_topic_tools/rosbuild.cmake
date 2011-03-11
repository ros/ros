if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_add_gtest(test/utest EXCLUDE_FROM_ALL test/utest.cpp)
rosbuild_add_executable(test/test_shapeshifter EXCLUDE_FROM_ALL test/test_shapeshifter.cpp)
rosbuild_add_gtest_build_flags(test/test_shapeshifter)
rosbuild_add_rostest(test/shapeshifter.test)
rosbuild_add_rostest(test/throttle.test)
rosbuild_add_rostest(test/drop.test)
rosbuild_add_rostest(test/relay.test)
rosbuild_add_rostest(test/mux.test)
rosbuild_add_rostest(test/switch_mux.test)
rosbuild_add_rostest(test/switch_mux_leading_slash.test)
rosbuild_add_rostest(test/switch_mux_none.test)
rosbuild_add_rostest(test/mux_add.test)
rosbuild_add_pyunit(test/args.py)

