rosbuild_add_boost_directories()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(tests/count_cores count_cores.cpp)
rosbuild_link_boost(tests/count_cores thread)
