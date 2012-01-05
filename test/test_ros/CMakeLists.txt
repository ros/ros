project(test_ros)

add_message_files(DIRECTORY msg
  FILES
  Arrays.msg
  CompositeA.msg CompositeB.msg Composite.msg
  Embed.msg Floats.msg Simple.msg
  TestArrays.msg
  TestHeader.msg
  TestPrimitives.msg
  TestString.msg
  TVals.msg
  )

generate_messages()

catkin_project(test_ros
  MSG_DIRS msg
  )

message("TODO: enable param server rostest")
#add_rostest(test/test-param-server.xml)
