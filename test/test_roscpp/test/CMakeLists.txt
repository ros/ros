rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/real_time_test.xml)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/sim_time_test.xml)

# Publish one message
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_once.xml)

# Publish a bunch of messages back to back
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast.xml)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_udp.xml)

# Publish a bunch of empty messages
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pub_onsub.xml)

# Publish a bunch of large messages back to back
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unadv.xml)

# Call a service
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call.xml)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv.xml)

# Repeatedly call ros::init() and ros::fini()
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_init_fini.xml)

# Test node inspection functionality
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/inspection.xml)

# Test logging functionality
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/log.xml)

# Test that advertising a service multiple times fails
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_multiple_providers.xml)

# Test command-line name remapping
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/name_remapping.xml)

# Test namespaces
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/namespaces.xml)

# Test params
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/params.xml)

# Test getting information from the master
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/get_master_information.xml)

# Test multiple subscriptions
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_subscriptions.xml)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong.xml)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large.xml)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/subscribe_self.xml)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/check_master.xml)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/handles.xml)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/timer_callbacks.xml)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/latching_publisher.xml)
