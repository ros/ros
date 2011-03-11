if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
# unit tests
rosbuild_add_pyunit(test/test_md5sums.py)
rosbuild_add_pyunit(test/test_roslib_genpy.py)
rosbuild_add_pyunit(test/test_roslib_gentools.py)
rosbuild_add_pyunit(test/test_roslib_message.py)
rosbuild_add_pyunit(test/test_roslib_msgs.py)

