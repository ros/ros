project(test_crosspackage)

find_package(catkin REQUIRED COMPONENTS genmsg)
catkin_project(${PROJECT_NAME})

add_message_files(
  DIRECTORY msg
  FILES
  TestMsgRename.msg
  TestSubMsgRename.msg
)

generate_messages()
