include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
rosbuild_add_rostest(test/test-param-server.xml)

