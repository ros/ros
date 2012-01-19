project(test_rostime)
catkin_project(test_rostime)
find_package(ROS COMPONENTS rostime rosunit)
include_directories(${ROS_INCLUDE_DIRS})
add_gtest(${PROJECT_NAME}-test_time test/time.cpp)
target_link_libraries(${PROJECT_NAME}-test_time ${ROS_LIBRARIES})

