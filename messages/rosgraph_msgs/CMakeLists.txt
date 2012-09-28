cmake_minimum_required(VERSION 2.8)
project(rosgraph_msgs)
find_package(catkin REQUIRED COMPONENTS genmsg std_msgs)
catkin_package()

add_message_files(DIRECTORY msg
  FILES
  Clock.msg
  Log.msg)

generate_messages(DEPENDENCIES std_msgs)
