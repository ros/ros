cmake_minimum_required(VERSION 2.8)
project(topic_tools)
find_package(catkin COMPONENTS cpp_common rosconsole roscpp rostest rostime rosunit xmlrpcpp)

catkin_package(
  LIBRARIES topic_tools
  INCLUDE_DIRS include
  DEPENDS rosconsole roscpp rostime xmlrpcpp
  )

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)
target_link_libraries(topic_tools ${catkin_LIBRARIES})

add_executable(switch_mux src/switch_mux.cpp)
target_link_libraries(switch_mux topic_tools ${catkin_LIBRARIES})

add_executable(mux src/mux.cpp)
target_link_libraries(mux topic_tools ${catkin_LIBRARIES})
add_dependencies(topic_tools topic_tools_gencpp)

add_executable(relay src/relay.cpp)
target_link_libraries(relay topic_tools ${catkin_LIBRARIES})

add_executable(drop src/drop.cpp)
target_link_libraries(drop topic_tools ${catkin_LIBRARIES})

add_executable(throttle src/throttle.cpp)
target_link_libraries(throttle topic_tools ${catkin_LIBRARIES})

add_service_files(DIRECTORY srv
  FILES
  MuxAdd.srv
  MuxDelete.srv
  MuxList.srv
  MuxSelect.srv)

generate_messages(DEPENDENCIES std_msgs)

install(TARGETS topic_tools switch_mux mux relay drop throttle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(PROGRAMS
  scripts/mux_add
  scripts/mux_delete
  scripts/mux_list
  scripts/mux_select
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Testing
catkin_add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest topic_tools)
endif()

if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(${PROJECT_NAME}-test_shapeshifter EXCLUDE_FROM_ALL test/test_shapeshifter.cpp)
  target_link_libraries(${PROJECT_NAME}-test_shapeshifter ${GTEST_LIBRARIES} topic_tools)
endif()
if(TARGET tests)
  add_dependencies(tests ${PROJECT_NAME}-test_shapeshifter)
endif()

add_rostest(test/shapeshifter.test)
add_rostest(test/throttle.test)
add_rostest(test/throttle_simtime.test)
add_rostest(test/drop.test)
add_rostest(test/relay.test)
## Latched test disabled until underlying issue in roscpp is resolved,
## #3385, #3434.
#rosbuild_add_rostest(test/relay_latched.test)
#rosbuild_add_rostest(test/mux.test)
#rosbuild_add_rostest(test/switch_mux.test)
#rosbuild_add_rostest(test/switch_mux_leading_slash.test)
#rosbuild_add_rostest(test/switch_mux_none.test)
#rosbuild_add_rostest(test/mux_add.test)

catkin_add_nosetests(test/args.py)
