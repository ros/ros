project(rosbag)
catkin_project(rosbag
  LIBRARIES rosbag
  INCLUDE_DIRS include
  )

find_package(ROS REQUIRED COMPONENTS cpp_common roscpp_serialization roscpp rosconsole xmlrpcpp topic_tools)
find_package(Boost REQUIRED COMPONENTS regex program_options)
find_package(BZip2 REQUIRED)
# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

include_directories(include ${BZIP2_INCLUDE_DIR} ${ROS_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS})
add_definitions(${BZIP2_DEFINITIONS})

add_library(rosbag SHARED
  src/bag.cpp src/buffer.cpp src/bz2_stream.cpp src/chunked_file.cpp
  src/message_instance.cpp src/player.cpp
  src/query.cpp src/recorder.cpp src/stream.cpp src/time_translator.cpp
  src/uncompressed_stream.cpp src/view.cpp)
target_link_libraries(rosbag ${ROS_LIBRARIES} ${Boost_LIBRARIES} ${BZIP2_LIBRARIES})

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
        LIBRARY DESTINATION lib)

rosbuild_download_test_data(http://pr.willowgarage.com/data/rosbag/test_indexed_1.2.bag
  test/test_indexed_1.2.bag 523cc0deb66269c84be4a7c605ff5eb3)
rosbuild_download_test_data(http://pr.willowgarage.com/data/rosbag/chatter_50hz.bag
  test/chatter_50hz.bag 00844db3729b1b5f34e767dc691bd531)
rosbuild_download_test_data(http://pr.willowgarage.com/data/rosbag/test_future_version_2.1.bag
  test/test_future_version_2.1.bag ddcb5e8711b16514eb640330ed93c332)

rosbuild_add_pyunit(test/test_bag.py)

rosbuild_add_gtest(test_bag test/test_bag.cpp)
target_link_libraries(test_bag rosbag)

rosbuild_add_rostest(test/rosbag_play.test)
rosbuild_add_rostest(test/play_play.test)
rosbuild_add_rostest(test/latched_pub.test)
rosbuild_add_rostest(test/latched_sub.test)
rosbuild_add_rostest(test/record_two_publishers.test)
add_dependencies(rostest_test_latched_sub.test rostest_test_latched_pub.test)
