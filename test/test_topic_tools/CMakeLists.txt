project(test_topic_tools)
catkin_project(test_topic_tools)
find_package(ROS COMPONENTS topic_tools)

include_directories(${ROS_INCLUDE_DIRS})
add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${ROS_LIBRARIES})

message("Enable test_topic_tools rostests")
#rosbuild_add_executable(test/test_shapeshifter EXCLUDE_FROM_ALL test/test_shapeshifter.cpp)
#rosbuild_add_gtest_build_flags(test/test_shapeshifter)
#
#rosbuild_add_rostest(test/shapeshifter.test)
#rosbuild_add_rostest(test/throttle.test)
#rosbuild_add_rostest(test/throttle_simtime.test)
#rosbuild_add_rostest(test/drop.test)
#rosbuild_add_rostest(test/relay.test)
## Latched test disabled until underlying issue in roscpp is resolved,
## #3385, #3434.
#rosbuild_add_rostest(test/relay_latched.test)
#rosbuild_add_rostest(test/mux.test)
#rosbuild_add_rostest(test/switch_mux.test)
#rosbuild_add_rostest(test/switch_mux_leading_slash.test)
#rosbuild_add_rostest(test/switch_mux_none.test)
#rosbuild_add_rostest(test/mux_add.test)

message("TODO: fix rosrun so that test_topic_tools pyunit will pass")
#add_pyunit(test/args.py)
