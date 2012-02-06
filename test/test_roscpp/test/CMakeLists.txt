find_package(ROS COMPONENTS rostest)

add_rostest(real_time_test.xml)
add_rostest(sim_time_test.xml)

# Publish one message
add_rostest(pubsub_once.xml)

# Publish a bunch of messages back to back
add_rostest(pubsub_n_fast.xml)
add_rostest(pubsub_n_fast_udp.xml)

# Publish a bunch of empty messages
add_rostest(pubsub_empty.xml)

# Publish only to the subscriber from the subscriber callback
add_rostest(pub_onsub.xml)

# Publish a bunch of large messages back to back
add_rostest(pubsub_n_fast_large_message.xml)

# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
add_rostest(pubsub_resub_once.xml)

# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
add_rostest(pubsub_unsub.xml)

# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
add_rostest(pubsub_unadv.xml)

# Call a service
add_rostest(service_call.xml)

# This test should be re-enabled after #2742 is fixed.
#add_rostest(service_deadlock.xml)

add_rostest(service_exception.xml)

add_rostest(service_call_unadv.xml)

# Repeatedly call ros::init() and ros::fini()
add_rostest(multiple_init_fini.xml)

# Test node inspection functionality
add_rostest(inspection.xml)

# Test that advertising a service multiple times fails
add_rostest(service_adv_multiple.xml)

# Test that the second node to advertise a service "wins"
add_rostest(service_multiple_providers.xml)

# Test namespaces
add_rostest(namespaces.xml)

# Test command-line name remapping
add_rostest(name_remapping.xml)

add_rostest(name_not_remappable.xml)

# Test command-line name remapping
add_rostest(name_remapping_ROS_NAMESPACE.xml)

# Test params
add_rostest(params.xml)

# Test getting information from the master
add_rostest(get_master_information.xml)

# Test multiple subscriptions
add_rostest(multiple_subscriptions.xml)

add_rostest(pingpong.xml)
add_rostest(pingpong_large.xml)

add_rostest(subscribe_self.xml)

add_rostest(check_master.xml)

add_rostest(handles.xml)
add_rostest(timer_callbacks.xml)
add_rostest(latching_publisher.xml)
add_rostest(loads_of_publishers.xml)
add_rostest(incrementing_sequence.xml)
add_rostest(subscription_callback_types.xml)
add_rostest(service_callback_types.xml)
add_rostest(intraprocess_subscriptions.xml)
add_rostest(nonconst_subscriptions.xml)
add_rostest(subscribe_retry_tcp.xml)
add_rostest(subscribe_star.xml)
add_rostest(parameter_validation.xml)

add_rostest(no_remappings.xml)
add_rostest(local_remappings.xml)
add_rostest(global_remappings.xml)
add_rostest(ns_node_remapping.xml)
add_rostest(search_param.xml)
