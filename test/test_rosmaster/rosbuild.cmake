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
rosbuild_add_rostest(test/rosmaster.test)
rosbuild_add_rostest(test/paramserver.test)
rosbuild_add_pyunit(test/test_rosmaster_registrations.py)
rosbuild_add_pyunit(test/test_rosmaster_paramserver.py)
rosbuild_add_pyunit(test/test_rosmaster_validators.py)

