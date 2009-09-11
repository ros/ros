rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/real_time_test.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/sim_time_test.xml)

# Publish one message
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_once.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/old_pubsub_once.xml)

# Publish a bunch of messages back to back
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_udp.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/old_pubsub_n_fast.xml)

# Publish a bunch of empty messages
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pub_onsub.xml)

# Publish a bunch of large messages back to back
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_large_message.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/old_pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_resub_once.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/old_pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unadv.xml)

# Call a service
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv.xml)

# Repeatedly call ros::init() and ros::fini()
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_init_fini.xml)

# Test node inspection functionality
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/inspection.xml)

# Test logging functionality
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/log.xml)

# Test that advertising a service multiple times fails
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_multiple_providers.xml)

# Test command-line name remapping
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/name_remapping.xml)

# Test namespaces
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/namespaces.xml)

# Test params
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/params.xml)

# Test getting information from the master
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/get_master_information.xml)

# Test multiple subscriptions
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_subscriptions.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/subscribe_self.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/check_master.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/handles.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/timer_callbacks.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/latching_publisher.xml)
