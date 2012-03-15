project(message_filters)
catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include
  LIBRARIES message_filters
  DEPENDS roscpp xmlrpcpp rosconsole
)
find_package(Boost COMPONENTS signals thread)
find_package(ROS COMPONENTS roscpp xmlrpcpp rosconsole)

include_directories(include ${ROS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/connection.cpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

# Ugly workaround for check_test_ran macro issue
#add_subdirectory(test)
find_package(ROS COMPONENTS rosunit rostest)

include_directories(${GTEST_INCLUDE_DIRS})

# ********** Tests **********
add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
target_link_libraries(${PROJECT_NAME}-msg_cache_unittest message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
target_link_libraries(${PROJECT_NAME}-test_synchronizer message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
target_link_libraries(${PROJECT_NAME}-test_exact_time_policy message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
target_link_libraries(${PROJECT_NAME}-test_simple message_filters ${GTEST_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
target_link_libraries(${PROJECT_NAME}-test_chain message_filters ${GTEST_LIBRARIES})

# Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
add_executable(${PROJECT_NAME}-time_sequencer_unittest EXCLUDE_FROM_ALL test/time_sequencer_unittest.cpp)
add_dependencies(tests ${PROJECT_NAME}-time_sequencer_unittest)
target_link_libraries(${PROJECT_NAME}-time_sequencer_unittest message_filters ${GTEST_LIBRARIES})
add_executable(${PROJECT_NAME}-test_subscriber EXCLUDE_FROM_ALL test/test_subscriber.cpp)
add_dependencies(tests ${PROJECT_NAME}-test_subscriber)
target_link_libraries(${PROJECT_NAME}-test_subscriber message_filters ${GTEST_LIBRARIES})

add_rostest(test/time_sequencer_unittest.xml)
add_rostest(test/test_subscriber.xml)