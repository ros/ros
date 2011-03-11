if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
rosbuild_add_pyunit(test/test_rosgraph_command_offline.py)
rosbuild_add_pyunit(test/test_rosgraph_masterapi_offline.py)
rosbuild_add_rostest(test/masterapi.test)

