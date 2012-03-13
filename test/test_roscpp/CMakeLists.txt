project(test_roscpp)
catkin_project(${PROJECT_NAME})

add_message_files(
  DIRECTORY msg
  FILES
  TestArray.msg
  TestEmpty.msg
  TestStringInt.msg
  TestWithHeader.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  BadTestStringString.srv
  TestStringString.srv
)
generate_messages(DEPENDENCIES std_msgs rosgraph_msgs)

find_package(ROS COMPONENTS roscpp xmlrpcpp rosconsole rostest)
include_directories(${ROS_INCLUDE_DIRS})
add_subdirectory(src)

add_rostest(test/real_time_test.xml)
add_rostest(test/sim_time_test.xml)

# Publish one message
add_rostest(test/pubsub_once.xml)

# Publish a bunch of messages back to back
add_rostest(test/pubsub_n_fast.xml)
add_rostest(test/pubsub_n_fast_udp.xml)

# Publish a bunch of empty messages
add_rostest(test/pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
add_rostest(test/pub_onsub.xml)

# Publish a bunch of large messages back to back
add_rostest(test/pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
add_rostest(test/pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
add_rostest(test/pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
add_rostest(test/pubsub_unadv.xml)

# Call a service
add_rostest(test/service_call.xml)

# This test should be re-enabled after #2742 is fixed.
#add_rostest(test/service_deadlock.xml)

add_rostest(test/service_exception.xml)

add_rostest(test/service_call_unadv.xml)

# Repeatedly call ros::init() and ros::fini()
add_rostest(test/multiple_init_fini.xml)

# Test node inspection functionality
add_rostest(test/inspection.xml)

# Test that advertising a service multiple times fails
add_rostest(test/service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
add_rostest(test/service_multiple_providers.xml)

# Test namespaces
add_rostest(test/namespaces.xml)

# Test command-line name remapping
add_rostest(test/name_remapping.xml)

add_rostest(test/name_not_remappable.xml)

# Test command-line name remapping
add_rostest(test/name_remapping_ROS_NAMESPACE.xml)

# Test params
add_rostest(test/params.xml)

# Test getting information from the master
add_rostest(test/get_master_information.xml)

# Test multiple subscriptions
add_rostest(test/multiple_subscriptions.xml)

add_rostest(test/pingpong.xml)
add_rostest(test/pingpong_large.xml)

add_rostest(test/subscribe_self.xml)

add_rostest(test/check_master.xml)

add_rostest(test/handles.xml)
add_rostest(test/timer_callbacks.xml)
add_rostest(test/latching_publisher.xml)
add_rostest(test/loads_of_publishers.xml)
add_rostest(test/incrementing_sequence.xml)
add_rostest(test/subscription_callback_types.xml)
add_rostest(test/service_callback_types.xml)
add_rostest(test/intraprocess_subscriptions.xml)
add_rostest(test/nonconst_subscriptions.xml)
add_rostest(test/subscribe_retry_tcp.xml)
add_rostest(test/subscribe_star.xml)
add_rostest(test/parameter_validation.xml)

add_rostest(test/no_remappings.xml)
add_rostest(test/local_remappings.xml)
add_rostest(test/global_remappings.xml)
add_rostest(test/ns_node_remapping.xml)
add_rostest(test/search_param.xml)
