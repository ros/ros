project(roslib)
find_package(catkin REQUIRED)
find_package(ROS REQUIRED COMPONENTS rospack rosunit)

include_directories(include ${Boost_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})

# Avoid a boost warning that pops up when using msvc compiler
if(MSVC)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif()

add_library(roslib src/package.cpp)
target_link_libraries(roslib ${Boost_LIBRARIES} ${ROS_LIBRARIES})

if(NOT (APPLE OR WIN32 OR MINGW))
  target_link_libraries(roslib rt)
endif()


#explicitly install library and includes
install(TARGETS roslib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)
install(PROGRAMS scripts/gendeps
        DESTINATION share/roslib/bin)

catkin_project(roslib
  LIBRARIES roslib
  INCLUDE_DIRS include
  )

#integration tests

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest roslib ${Boost_LIBRARIES} ${ROS_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_package test/package.cpp)
target_link_libraries(${PROJECT_NAME}-test_package roslib ${ROS_LIBRARIES})
