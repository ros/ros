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
rosbuild_download_test_data(http://pr.willowgarage.com/data/rosrecord/rosrecord_rename_test.bag test/rosrecord_rename_test.bag 09c6e106c77b06ed3e836a5280c8a7c2)
rosbuild_download_test_data(http://pr.willowgarage.com/data/test_rosrecord/hello_world.bag test/hello_world.bag 95b2aecfd2069c4acb7a29357c7d9c41)
rosbuild_add_pyunit(test/test_rosrecord_py.py)
rosbuild_add_rostest(test/rename_test.xml)

rosbuild_add_gtest(test/test_recorder_player test/test_recorder_player.cpp)

# Remove test until we resolve details of making them pass deterministically
rosbuild_add_rostest_future(test/random_pub_sub.xml)
rosbuild_add_rostest_future(test/random_pub_record.xml)
rosbuild_add_rostest_future(test/random_play_sub.xml)
