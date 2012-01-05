project(rosout)

rosbuild_add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${ROS_LIBRARIES})

install(FILES manifest.xml
  DESTINATION share/rosout
  )
install(TARGETS rosout
  RUNTIME DESTINATION share/rosout/bin)
