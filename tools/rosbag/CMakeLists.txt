project(rosbag)
catkin_project(rosbag
  LIBRARIES rosbag
  INCLUDE_DIRS include
  )

find_package(catkin REQUIRED COMPONENTS cpp_common roscpp_serialization roscpp rosconsole xmlrpcpp topic_tools rosunit rostest rosgraph_msgs)
find_package(Boost REQUIRED COMPONENTS regex program_options)
find_package(BZip2 REQUIRED)
# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

include_directories(include ${BZIP2_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
add_definitions(${BZIP2_DEFINITIONS})

add_library(rosbag 
  src/bag.cpp src/buffer.cpp src/bz2_stream.cpp src/chunked_file.cpp
  src/message_instance.cpp src/player.cpp
  src/query.cpp src/recorder.cpp src/stream.cpp src/time_translator.cpp
  src/uncompressed_stream.cpp src/view.cpp)
target_link_libraries(rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BZIP2_LIBRARIES})

add_executable(record src/record.cpp)
target_link_libraries(record rosbag)

add_executable(play src/play.cpp)
target_link_libraries(play rosbag)

#rosbuild_add_executable(example_write examples/write.cpp)
#target_link_libraries(example_write rosbag)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
install(TARGETS rosbag record play
        RUNTIME DESTINATION share/rosbag/bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
install(PROGRAMS
        scripts/bag2png.py
        scripts/bagsort.py
        scripts/fastrebag.py
        scripts/fixbag.py
        scripts/fixbag_batch.py
        scripts/fix_md5sums.py
        scripts/fix_moved_messages.py
        scripts/fix_msg_defs.py
        scripts/makerule.py
        scripts/savemsg.py
        scripts/topic_renamer.py
        DESTINATION share/rosbag/bin
        )

download_test_data(http://pr.willowgarage.com/data/rosbag/test_indexed_1.2.bag
  test/test_indexed_1.2.bag 523cc0deb66269c84be4a7c605ff5eb3)
download_test_data(http://pr.willowgarage.com/data/rosbag/chatter_50hz.bag
  test/chatter_50hz.bag 00844db3729b1b5f34e767dc691bd531)
download_test_data(http://pr.willowgarage.com/data/rosbag/test_future_version_2.1.bag
  test/test_future_version_2.1.bag ddcb5e8711b16514eb640330ed93c332)

add_pyunit(test/test_bag.py)

configure_file(test/test_bag.cpp.in 
               ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
add_gtest(test_bag ${PROJECT_BINARY_DIR}/test/test_bag.cpp)
if(TARGET test_bag)
  target_link_libraries(test_bag rosbag)
endif()

configure_file(test/play_play.test.in 
               ${PROJECT_BINARY_DIR}/test/play_play.test)
add_rostest(${PROJECT_BINARY_DIR}/test/play_play.test)
configure_file(test/rosbag_play.test.in 
               ${PROJECT_BINARY_DIR}/test/rosbag_play.test)
add_rostest(${PROJECT_BINARY_DIR}/test/rosbag_play.test)

add_rostest(test/latched_pub.test)
add_rostest(test/latched_sub.test)
add_rostest(test/record_two_publishers.test)
add_rostest(test/record_one_publisher_two_topics.test)

include_directories(${GTEST_INCLUDE_DIRS})
add_executable(double_pub EXCLUDE_FROM_ALL test/double_pub.cpp)
target_link_libraries(double_pub ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
if(TARGET tests)
  add_dependencies(tests double_pub)
endif()
