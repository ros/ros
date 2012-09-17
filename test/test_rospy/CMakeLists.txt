cmake_minimum_required(VERSION 2.8)
project(test_rospy)
find_package(catkin REQUIRED COMPONENTS genmsg test_ros rosunit rostest)
catkin_project(${PROJECT_NAME})

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
add_service_files(DIRECTORY srv
  FILES
  ConstantsMultiplex.srv
  EmptyReqSrv.srv
  EmptyRespSrv.srv
  EmptySrv.srv
  ListReturn.srv
  MultipleAddTwoInts.srv
  StringString.srv
  TransitiveSrv.srv
  )

generate_messages(DEPENDENCIES std_msgs test_ros)

catkin_add_nosetests(test/unit)

# integration tests
# - accidentally wrote two integration tests for client param API. They are different enough that it's no harm in having both
add_rostest(test/rostest/client-param-server.test)
add_rostest(test/rostest/client-param-api-2.test)
add_rostest(test/rostest/services.test)
add_rostest(test/rostest/deregister.test)
add_rostest(test/rostest/pubsub-order.test)
add_rostest(test/rostest/embed-msg.test)
add_rostest(test/rostest/rospy.test)
add_rostest(test/rostest/rospy_sim_time.test)
add_rostest(test/rostest/node.test)
add_rostest(test/rostest/latch.test)
add_rostest(test/rostest/on_shutdown.test)
