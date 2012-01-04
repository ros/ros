project(roslib)
find_package(ROS COMPONENTS catkin rospack)

include_directories(include ${Boost_INCLUDE_DIRS} ${ROS_INCLUDE_DIRS})

# Avoid a boost warning that pops up when using msvc compiler
if(MSVC)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
endif()

add_library(roslib SHARED src/package.cpp)
target_link_libraries(roslib ${Boost_LIBRARIES} ${ROS_LIBRARIES})

if(NOT (APPLE OR WIN32 OR MINGW))
  target_link_libraries(roslib rt)
endif()


#explicitly install library and includes
install(TARGETS roslib
  DESTINATION lib
  )

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  )

catkin_project(roslib
  VERSION 0.0.1
  LIBRARIES roslib
  INCLUDE_DIRS include
  )

catkin_export_python(src)
# We have to set ros_PYTHONPATH to get the 'ros' module copied into the
# buildspace.  Not sure what will happen if multiple packages do this...
# set(ros_PYTHONPATH src)


#integration tests

add_gtest(${PROJECT_NAME}-utest test/utest.cpp)
target_link_libraries(${PROJECT_NAME}-utest ${Boost_LIBRARIES} ${ROS_LIBRARIES})

add_gtest(${PROJECT_NAME}-test_package test/package.cpp)
target_link_libraries(${PROJECT_NAME}-test_package ${ROS_LIBRARIES})
