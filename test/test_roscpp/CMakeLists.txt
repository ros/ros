project(test_roscpp)
catkin_project(${PROJECT_NAME})

add_message_files(
  DIRECTORY msg
  FILES
  TestArray.msg
  TestEmpty.msg
  TestStringInt.msg
  TestWithHeader.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  BadTestStringString.srv
  TestStringString.srv
)
generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)

find_package(ROS COMPONENTS roscpp)

add_subdirectory(src)
#add_subdirectory(test)

