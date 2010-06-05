cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_pyunit(test/migrate_test.py TIMEOUT 120.0)
rosbuild_add_rostest(test/random_record.xml)
rosbuild_add_rostest(test/random_play.xml)
ADD_DEPENDENCIES(rostest_test_random_play.xml rostest_test_random_record.xml)
rosbuild_add_rostest(test/random_play_sim.xml)
ADD_DEPENDENCIES(rostest_test_random_play_sim.xml rostest_test_random_record.xml)

