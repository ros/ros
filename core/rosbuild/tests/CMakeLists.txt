rosbuild_add_boost_directories()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
# Can't build this because it needs Boost, and you need rosdep to install
# Boost, and you need rosbuild to build rosdep, etc., etc., #1865.
#add_executable(tests/count_cores count_cores.cpp)
#rosbuild_link_boost(tests/count_cores thread)
