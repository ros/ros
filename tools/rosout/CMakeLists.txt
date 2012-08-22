project(rosout)
catkin_project(rosout)

add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${catkin_LIBRARIES})
set_target_properties(rosout PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${catkin_BUILD_PREFIX}/lib/rosout)

install(TARGETS rosout
  RUNTIME DESTINATION share/rosout/bin)
