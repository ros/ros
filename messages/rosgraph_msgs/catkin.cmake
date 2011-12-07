cmake_minimum_required(VERSION 2.8)
project(rosgraph_msgs)

find_package(catkin)
find_package(genmsg)
find_package(std_msgs)

add_message_files(DIRECTORY msg
  FILES Clock.msg Log.msg
  )

generate_messages(DEPENDENCIES std_msgs)

install_cmake_infrastructure(rosgraph_msgs
  VERSION 0.0.1
  MSG_DIRS msg
  )

