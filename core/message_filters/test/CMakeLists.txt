set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********
rospack_add_gtest(test/msg_cache_unittest msg_cache_unittest.cpp)
target_link_libraries(test/msg_cache_unittest message_filters)

rospack_add_gtest(test/time_synchronizer_unittest time_synchronizer_unittest.cpp)
target_link_libraries(test/time_synchronizer_unittest message_filters)

# Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
rospack_add_executable(test/time_sequencer_unittest EXCLUDE_FROM_ALL time_sequencer_unittest.cpp)
rospack_declare_test(test/time_sequencer_unittest)
rospack_add_gtest_build_flags(test/time_sequencer_unittest)
target_link_libraries(test/time_sequencer_unittest message_filters)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/time_sequencer_unittest.xml)