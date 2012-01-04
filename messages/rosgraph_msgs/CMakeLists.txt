project(rosgraph_msgs)

add_message_files(DIRECTORY msg
  FILES Clock.msg Log.msg
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_project(rosgraph_msgs
  MSG_DIRS msg
  )
