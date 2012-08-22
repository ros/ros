project(topic_tools)
catkin_project(topic_tools
  LIBRARIES topic_tools
  INCLUDE_DIRS include
  DEPENDS roscpp rosconsole xmlrpcpp rostime
  )
include_directories(include)
find_package(catkin COMPONENTS cpp_common roscpp rosconsole xmlrpcpp rosunit rostest rostime)
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
  FILES MuxAdd.srv MuxDelete.srv MuxList.srv MuxSelect.srv
  )

generate_messages(DEPENDENCIES std_msgs)

install(TARGETS topic_tools switch_mux mux relay drop throttle
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION share/topic_tools/bin
    )

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(PROGRAMS
        scripts/mux_add
        scripts/mux_delete
        scripts/mux_list
        scripts/mux_select
        DESTINATION share/topic_tools/scripts)

#Testing
add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
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

add_pyunit(test/args.py)
