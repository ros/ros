project(test_ros)

find_package(catkin COMPONENTS genmsg std_msgs rosgraph_msgs rostest)

add_message_files(DIRECTORY msg
  FILES
  Arrays.msg
  CompositeA.msg CompositeB.msg Composite.msg
  Embed.msg Floats.msg Simple.msg
  RosmsgA.msg
  RosmsgB.msg
  RosmsgC.msg
  TestArrays.msg
  TestHeader.msg
  TestPrimitives.msg
  TestString.msg
  TVals.msg
  )
add_service_files(DIRECTORY srv
  FILES
  AddTwoInts.srv
  RossrvA.srv
  RossrvB.srv
)

generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)

catkin_project(test_ros)

add_rostest(test/test-param-server.xml)

# make the talker/listener available for roslaunch example
install(DIRECTORY nodes
  DESTINATION share/test_ros
  PATTERN ".svn" EXCLUDE
  )
