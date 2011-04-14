include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
rosbuild_add_executable(pointcloud_serdes pointcloud_serdes.cpp)
#rosbuild_add_compile_flags(pointcloud_serdes "-O3 -funroll-loops")
#rosbuild_add_compile_flags(pointcloud_serdes "-march=prescott")

