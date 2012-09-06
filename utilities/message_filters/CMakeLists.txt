project(message_filters)
find_package(catkin REQUIRED COMPONENTS roscpp xmlrpcpp rosconsole)
catkin_project(${PROJECT_NAME}
  INCLUDE_DIRS include
  LIBRARIES message_filters
  DEPENDS roscpp xmlrpcpp rosconsole
)
find_package(Boost REQUIRED COMPONENTS signals thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/connection.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PROJECT_LIB_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PROJECT_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# Ugly workaround for check_test_ran macro issue
#add_subdirectory(test)
find_package(catkin COMPONENTS rosunit rostest)

include_directories(${GTEST_INCLUDE_DIRS})

# ********** Tests **********
catkin_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
  target_link_libraries(${PROJECT_NAME}-msg_cache_unittest message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
  target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
if(TARGET ${PROJECT_NAME}-test_synchronizer)
  target_link_libraries(${PROJECT_NAME}-test_synchronizer message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
  target_link_libraries(${PROJECT_NAME}-test_exact_time_policy message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
  target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
if(TARGET ${PROJECT_NAME}-test_simple)
  target_link_libraries(${PROJECT_NAME}-test_simple message_filters ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
if(TARGET ${PROJECT_NAME}-test_chain)
  target_link_libraries(${PROJECT_NAME}-test_chain message_filters ${GTEST_LIBRARIES})
endif()

# Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
add_executable(${PROJECT_NAME}-time_sequencer_unittest EXCLUDE_FROM_ALL test/time_sequencer_unittest.cpp)
if(TARGET tests)
  add_dependencies(tests ${PROJECT_NAME}-time_sequencer_unittest)
endif()
target_link_libraries(${PROJECT_NAME}-time_sequencer_unittest message_filters ${GTEST_LIBRARIES})
add_executable(${PROJECT_NAME}-test_subscriber EXCLUDE_FROM_ALL test/test_subscriber.cpp)
if(TARGET tests)
  add_dependencies(tests ${PROJECT_NAME}-test_subscriber)
endif()
target_link_libraries(${PROJECT_NAME}-test_subscriber message_filters ${GTEST_LIBRARIES})

add_rostest(test/time_sequencer_unittest.xml)
add_rostest(test/test_subscriber.xml)