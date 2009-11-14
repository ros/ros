cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_gensrv()

rosbuild_add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)

rosbuild_add_executable(bin/switch_mux src/switch_mux.cpp)
target_link_libraries(bin/switch_mux topic_tools)

rosbuild_add_executable(bin/mux src/mux.cpp)
target_link_libraries(bin/mux topic_tools)

rosbuild_add_executable(bin/relay src/relay.cpp)
target_link_libraries(bin/relay topic_tools)

rosbuild_add_executable(bin/drop src/drop.cpp)
target_link_libraries(bin/drop topic_tools)

#rospack_add_executable(demux demux.cpp)
#target_link_libraries(demux topic_tools)

rosbuild_add_executable(bin/throttle src/throttle.cpp)
target_link_libraries(bin/throttle topic_tools)

rosbuild_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest topic_tools)

rosbuild_add_rostest(test/throttle.test)
rosbuild_add_rostest(test/drop.test)
rosbuild_add_rostest(test/relay.test)
rosbuild_add_rostest(test/mux.test)
rosbuild_add_rostest(test/switch_mux.test)
# Can't run this one yet, because hztest won't check for 0Hz output, # 2024
#rosbuild_add_rostest(test/switch_mux_none.test)

rosbuild_add_pyunit(test/args.py)
