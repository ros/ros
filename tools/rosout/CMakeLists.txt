project(rosout)
catkin_project(rosout)

add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${ROS_LIBRARIES})

install(TARGETS rosout
  RUNTIME DESTINATION share/rosout/bin)
