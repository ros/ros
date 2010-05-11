cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo) 
rosbuild_init()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# unit tests
rosbuild_download_test_data(http://pr.willowgarage.com/data/rosrecord/rosrecord_rename_test.indexed.bag test/rosrecord_rename_test.bag 757ab53c4876d21f6d60b9a3e0efb23e)
rosbuild_download_test_data(http://pr.willowgarage.com/data/test_rosrecord/hello_world.indexed.bag test/hello_world.bag 10fa1e9a4184ddefb69d74a7d9021fc7)
rosbuild_add_pyunit(test/test_rosrecord_py.py)
rosbuild_add_rostest(test/rename_test.xml)

rosbuild_add_boost_directories()
rosbuild_add_gtest(test/test_recorder_player test/test_recorder_player.cpp)
rosbuild_link_boost(test/test_recorder_player system)

# Remove test until we resolve details of making them pass deterministically
rosbuild_add_rostest(test/random_record.xml)
rosbuild_add_rostest(test/random_play.xml)
ADD_DEPENDENCIES(rostest_test_random_play.xml rostest_test_random_record.xml)
rosbuild_add_rostest(test/random_play_sim.xml)
ADD_DEPENDENCIES(rostest_test_random_play_sim.xml rostest_test_random_record.xml)

