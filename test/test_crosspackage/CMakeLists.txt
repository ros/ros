project(test_crosspackage)
catkin_project(test_crosspackage)

add_message_files(
  DIRECTORY msg
  FILES
  TestMsgRename.msg
  TestSubMsgRename.msg
)

generate_messages()
