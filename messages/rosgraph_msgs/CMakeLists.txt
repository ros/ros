project(rosgraph_msgs)

add_message_files(DIRECTORY msg
  FILES Clock.msg Log.msg
  )

generate_messages(DEPENDENCIES std_msgs)

install_cmake_infrastructure(rosgraph_msgs
  VERSION 0.0.1
  MSG_DIRS msg
  )

enable_python(rosgraph_msgs)