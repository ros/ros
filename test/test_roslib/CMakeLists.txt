project(test_roslib)
find_package(catkin)
find_package(Boost COMPONENTS thread)
find_package(ROS COMPONENTS roslib)

#integration tests

include_directories(${Boost_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})
add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${Boost_LIBRARIES} ${ROS_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_package test/package.cpp)
target_link_libraries(${PROJECT_NAME}-test_package ${ROS_LIBRARIES})
