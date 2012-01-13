find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# ********** Tests **********
add_gtest(${PROJECT_NAME}-msg_cache_unittest msg_cache_unittest.cpp)
target_link_libraries(${PROJECT_NAME}-msg_cache_unittest message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-time_synchronizer_unittest time_synchronizer_unittest.cpp)
target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_synchronizer test_synchronizer.cpp)
target_link_libraries(${PROJECT_NAME}-test_synchronizer message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_exact_time_policy test_exact_time_policy.cpp)
target_link_libraries(${PROJECT_NAME}-test_exact_time_policy message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_approximate_time_policy test_approximate_time_policy.cpp)
target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_simple test_simple.cpp)
target_link_libraries(${PROJECT_NAME}-test_simple message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_chain test_chain.cpp)
target_link_libraries(${PROJECT_NAME}-test_chain message_filters ${GTEST_LIBRARIES})

# Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
add_executable(${PROJECT_NAME}-time_sequencer_unittest EXCLUDE_FROM_ALL time_sequencer_unittest.cpp)
add_dependencies(tests ${PROJECT_NAME}-time_sequencer_unittest)
target_link_libraries(${PROJECT_NAME}-time_sequencer_unittest message_filters ${GTEST_LIBRARIES})
add_rostest(time_sequencer_unittest.xml)
add_executable(${PROJECT_NAME}-test_subscriber EXCLUDE_FROM_ALL test_subscriber.cpp)
add_dependencies(tests ${PROJECT_NAME}-test_subscriber)
target_link_libraries(${PROJECT_NAME}-test_subscriber message_filters ${GTEST_LIBRARIES})
add_rostest(test_subscriber.xml)
