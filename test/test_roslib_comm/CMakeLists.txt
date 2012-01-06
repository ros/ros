cmake_minimum_required(VERSION 2.8)
project(test_roslib_comm)
find_package(catkin)

add_message_files(DIRECTORY msg
  FILES
  ArrayOfMsgs.msg
  FieldNameChange1.msg
  FieldNameChange2.msg
  FillEmbedTime.msg
  FillSimple.msg
  HeaderTest.msg
  SameSubMsg1.msg
  SameSubMsg2.msg
  SameSubMsg3.msg
  TypeNameChange1.msg
  TypeNameChange2.msg
  TypeNameChangeArray1.msg
  TypeNameChangeArray2.msg
  TypeNameChangeComplex1.msg
  TypeNameChangeComplex2.msg
  )
generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)

catkin_project(test_roslib_comm
  MSG_DIRS msg
  )
