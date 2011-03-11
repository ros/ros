if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
rosbuild_add_pyunit(test/test_rostopic_command_line_offline.py)
rosbuild_add_pyunit(test/test_rostopic_unit.py)
rosbuild_add_rostest(test/rostopic.test)

