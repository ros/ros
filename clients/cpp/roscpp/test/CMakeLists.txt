catkin_add_gtest(${PROJECT_NAME}-test_version test_version.cpp)
if(TARGET ${PROJECT_NAME}-test_version)
  target_link_libraries(${PROJECT_NAME}-test_version)
endif()

# WARNING test_spinners is not actually run.  This is because this
# infrastructure won't let me pass arguments from here through to
# these test units.  otherwise one would have to build ten executables.
# also the output of test_spinners has to be visually inspected
# because there is no automatic way to ensure that an error condition
# provokes a particular message.
if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(${PROJECT_NAME}-test_spinners EXCLUDE_FROM_ALL test_spinners.cpp)
  add_dependencies(tests ${PROJECT_NAME}-test_spinners) 
endif()
if(TARGET ${PROJECT_NAME}-test_spinners)
  target_link_libraries(${PROJECT_NAME}-test_spinners roscpp ${GTEST_LIBRARIES})
endif()

catkin_add_gtest(${PROJECT_NAME}-test_header test_header.cpp)
if(TARGET ${PROJECT_NAME}-test_header)
  target_link_libraries(${PROJECT_NAME}-test_header roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_poll_set test_poll_set.cpp)
if(TARGET ${PROJECT_NAME}-test_poll_set)
  target_link_libraries(${PROJECT_NAME}-test_poll_set roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_transport_tcp test_transport_tcp.cpp)
if(TARGET ${PROJECT_NAME}-test_transport_tcp)
  target_link_libraries(${PROJECT_NAME}-test_transport_tcp roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_subscription_queue test_subscription_queue.cpp)
if(TARGET ${PROJECT_NAME}-test_subscription_queue)
  target_link_libraries(${PROJECT_NAME}-test_subscription_queue roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_callback_queue test_callback_queue.cpp)
if(TARGET ${PROJECT_NAME}-test_callback_queue)
  target_link_libraries(${PROJECT_NAME}-test_callback_queue roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_names test_names.cpp)
if(TARGET ${PROJECT_NAME}-test_names)
  target_link_libraries(${PROJECT_NAME}-test_names roscpp)
endif()

catkin_add_gtest(${PROJECT_NAME}-test_args test_args.cpp)
if(TARGET ${PROJECT_NAME}-test_args)
  target_link_libraries(${PROJECT_NAME}-test_args roscpp)
endif()

if(GTEST_FOUND)
  add_subdirectory(src)
endif()

add_rostest(launch/real_time_test.xml)
add_rostest(launch/sim_time_test.xml)

# Publish one message
add_rostest(launch/pubsub_once.xml)

# Publish a bunch of messages back to back
add_rostest(launch/pubsub_n_fast.xml)
add_rostest(launch/pubsub_n_fast_udp.xml)

# Publish a bunch of empty messages
add_rostest(launch/pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
add_rostest(launch/pub_onsub.xml)

# Publish a bunch of large messages back to back
add_rostest(launch/pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
add_rostest(launch/pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
add_rostest(launch/pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
add_rostest(launch/pubsub_unadv.xml)

# Call a service
add_rostest(launch/service_call.xml)

# This test should be re-enabled after #2742 is fixed.
#add_rostest(launch/service_deadlock.xml)

add_rostest(launch/service_exception.xml)

add_rostest(launch/service_call_unadv.xml)

# Repeatedly call ros::init() and ros::fini()
add_rostest(launch/multiple_init_fini.xml)

# Test node inspection functionality
add_rostest(launch/inspection.xml)

# Test that advertising a service multiple times fails
add_rostest(launch/service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
add_rostest(launch/service_multiple_providers.xml)

# Test namespaces
add_rostest(launch/namespaces.xml)

# Test command-line name remapping
add_rostest(launch/name_remapping.xml)

add_rostest(launch/name_not_remappable.xml)

# Test command-line name remapping
add_rostest(launch/name_remapping_ROS_NAMESPACE.xml)

# Test params
add_rostest(launch/params.xml)

# Test getting information from the master
add_rostest(launch/get_master_information.xml)

# Test multiple subscriptions
add_rostest(launch/multiple_subscriptions.xml)

add_rostest(launch/pingpong.xml)
add_rostest(launch/pingpong_large.xml)

add_rostest(launch/subscribe_self.xml)

add_rostest(launch/check_master.xml)

add_rostest(launch/handles.xml)
add_rostest(launch/timer_callbacks.xml)
add_rostest(launch/latching_publisher.xml)
add_rostest(launch/loads_of_publishers.xml)
add_rostest(launch/incrementing_sequence.xml)
add_rostest(launch/subscription_callback_types.xml)
add_rostest(launch/service_callback_types.xml)
add_rostest(launch/intraprocess_subscriptions.xml)
add_rostest(launch/nonconst_subscriptions.xml)
add_rostest(launch/subscribe_retry_tcp.xml)
add_rostest(launch/subscribe_star.xml)
add_rostest(launch/parameter_validation.xml)

add_rostest(launch/no_remappings.xml)
add_rostest(launch/local_remappings.xml)
add_rostest(launch/global_remappings.xml)
add_rostest(launch/ns_node_remapping.xml)
add_rostest(launch/search_param.xml)
