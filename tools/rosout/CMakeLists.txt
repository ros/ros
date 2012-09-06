project(rosout)
catkin_project(rosout)

add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${catkin_LIBRARIES})

install(TARGETS rosout
  RUNTIME DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION})
