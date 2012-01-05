cmake_minimum_required(VERSION 2.8)
project(test_rospy)
find_package(catkin)

# messages are for integration testing only
add_message_files(DIRECTORY msg
  FILES
  ArrayVal.msg
  EmbedTest.msg
  Floats.msg
  HeaderHeaderVal.msg
  HeaderVal.msg
  PythonKeyword.msg
  TestConstants.msg
  TestFixedArray.msg
  TransitiveImport.msg
  TransitiveMsg1.msg
  TransitiveMsg2.msg
  Val.msg
  )
generate_messages()

catkin_project(test_rospy
  MSGS_DIRS msg
  )

message("TODO: enable rospy rostests")

# integration tests
# - accidentally wrote two integration tests for client param API. They are different enough that it's no harm in having both
#add_rostest(test/client-param-server.test)
#add_rostest(test/client-param-api-2.test)
#add_rostest(test/services.test)
#add_rostest(test/deregister.test)
#add_rostest(test/pubsub-order.test)
#add_rostest(test/embed-msg.test)
#add_rostest(test/rospy.test)
#add_rostest(test/rospy_sim_time.test)
#add_rostest(test/node.test)
#add_rostest(test/latch.test)
#add_rostest(test/on_shutdown.test)
