project(test_rostime)

find_package(catkin COMPONENTS cpp_common rostime rosunit)

include_directories(${catkin_INCLUDE_DIRS})

catkin_add_gtest(${PROJECT_NAME}-test_time test/time.cpp)
if(TARGET ${PROJECT_NAME}-test_time)
  target_link_libraries(${PROJECT_NAME}-test_time ${catkin_LIBRARIES})
endif()
