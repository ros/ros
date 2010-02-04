set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rosbuild_add_gtest(test/msg_cache_unittest msg_cache_unittest.cpp)
target_link_libraries(test/msg_cache_unittest message_filters)

rosbuild_add_gtest(test/time_synchronizer_unittest time_synchronizer_unittest.cpp)
target_link_libraries(test/time_synchronizer_unittest message_filters)

# Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
rosbuild_add_executable(test/time_sequencer_unittest EXCLUDE_FROM_ALL time_sequencer_unittest.cpp)
rosbuild_declare_test(test/time_sequencer_unittest)
rosbuild_add_gtest_build_flags(test/time_sequencer_unittest)
target_link_libraries(test/time_sequencer_unittest message_filters)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/time_sequencer_unittest.xml)

rosbuild_add_executable(test/test_subscriber EXCLUDE_FROM_ALL test_subscriber.cpp)
rosbuild_declare_test(test/test_subscriber)
rosbuild_add_gtest_build_flags(test/test_subscriber)
target_link_libraries(test/test_subscriber message_filters)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test_subscriber.xml)
