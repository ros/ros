rospack_add_executable(publish_n_fast publish_n_fast.cpp)
rospack_declare_test(publish_n_fast)
target_link_libraries(publish_n_fast ros)

rospack_add_executable(subscribe_self subscribe_self.cpp)
rospack_add_gtest_build_flags(subscribe_self)
target_link_libraries(subscribe_self ros)

rospack_add_executable(pub_sub pub_sub.cpp)
rospack_declare_test(pub_sub)
target_link_libraries(pub_sub ros)

rospack_add_executable(sub_pub sub_pub.cpp)
rospack_add_gtest_build_flags(sub_pub)
target_link_libraries(sub_pub ros)

rospack_add_executable(publish_empty publish_empty.cpp)
rospack_declare_test(publish_empty)
target_link_libraries(publish_empty ros)

rospack_add_executable(publish_onsub publish_onsub.cpp)
rospack_declare_test(publish_onsub)
target_link_libraries(publish_onsub ros)

rospack_add_executable(subscribe_n_fast subscribe_n_fast.cpp)
rospack_add_gtest_build_flags(subscribe_n_fast)
target_link_libraries(subscribe_n_fast ros)

rospack_add_executable(subscribe_empty subscribe_empty.cpp)
rospack_add_gtest_build_flags(subscribe_empty)
target_link_libraries(subscribe_empty ros)

rospack_add_executable(subscribe_resubscribe subscribe_resubscribe.cpp)
rospack_add_gtest_build_flags(subscribe_resubscribe)
target_link_libraries(subscribe_resubscribe ros)

rospack_add_executable(subscribe_unsubscribe subscribe_unsubscribe.cpp)
rospack_add_gtest_build_flags(subscribe_unsubscribe)
target_link_libraries(subscribe_unsubscribe ros)

rospack_add_executable(publish_unadvertise publish_unadvertise.cpp)
rospack_add_gtest_build_flags(publish_unadvertise)
target_link_libraries(publish_unadvertise ros)

rospack_add_executable(subscribe_unsubscribe_repeatedly subscribe_unsubscribe_repeatedly.cpp)
rospack_declare_test(subscribe_unsubscribe_repeatedly)
target_link_libraries(subscribe_unsubscribe_repeatedly ros)

rospack_add_executable(publish_constantly publish_constantly.cpp)
rospack_declare_test(publish_constantly)
target_link_libraries(publish_constantly ros)

rospack_add_executable(shutdown_master shutdown_master.cpp)
rospack_declare_test(shutdown_master)
target_link_libraries(shutdown_master ros)

rospack_add_executable(param_update_test param_update_test.cpp)
rospack_declare_test(param_update_test)
target_link_libraries(param_update_test ros)

 
##################
rospack_add_executable(real_time_test real_time_test.cpp)
rospack_declare_test(real_time_test)
rospack_add_gtest_build_flags(real_time_test)
target_link_libraries(real_time_test ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/real_time_test.xml)
##################

##################
rospack_add_executable(sim_time_test sim_time_test.cpp)
rospack_declare_test(sim_time_test)
rospack_add_gtest_build_flags(sim_time_test)
target_link_libraries(sim_time_test ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/sim_time_test.xml)
##################

##################

##################
# Publish one message
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_once.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_once_nothread.xml)
##################

##################
# Publish a bunch of messages back to back
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_nothread.xml)
##################

##################
# Publish a bunch of empty messages
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_empty.xml)
##################

##################
# Publish only to the subscriber from the subscriber callback
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pub_onsub.xml)
##################

##################
# Publish a bunch of large messages back to back
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_large_message.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_large_message_nothread.xml)
##################

##################
# Subscribe, listen, unsubscribe, re-subscribe to a different topic, listen
# again
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_resub_once.xml)
#rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_n_fast_large_message_nothread.xml)
##################

##################
# Subscribe and unsubscribe repeatedly, ensuring that callbacks don't get
# called when not subscribed.
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unsub.xml)
##################

##################
# Advertise, then unadvertise, and ensure that subscriber callback doesn't
# get invoked afterward, while a subscriber is constantly subscribing and
# unsubscribing
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unadv.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pubsub_unadv_nothread.xml)
##################

##################
# Call a service
rospack_add_executable(service_adv service_adv.cpp)
rospack_declare_test(service_adv)
target_link_libraries(service_adv ros)

rospack_add_executable(service_adv_unadv service_adv_unadv.cpp)
rospack_declare_test(service_adv_unadv)
rospack_add_gtest_build_flags(service_adv_unadv)
target_link_libraries(service_adv_unadv ros)

rospack_add_executable(service_call service_call.cpp)
rospack_add_gtest_build_flags(service_call)
target_link_libraries(service_call ros)

rospack_add_executable(service_call_repeatedly service_call_repeatedly.cpp)
rospack_declare_test(service_call_repeatedly)
target_link_libraries(service_call_repeatedly ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv_nothread.xml)
##################

##################
# Repeatedly call ros::init() and ros::fini()
rospack_add_executable(multiple_init_fini multiple_init_fini.cpp)
rospack_add_gtest_build_flags(multiple_init_fini)
target_link_libraries(multiple_init_fini ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_init_fini.xml)
##################

##################
# Test node inspection functionality
rospack_add_executable(inspection inspection.cpp)
rospack_add_gtest_build_flags(inspection)
target_link_libraries(inspection ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/inspection.xml)
##################

##################
# Test logging functionality
rospack_add_executable(log log.cpp)
rospack_add_gtest_build_flags(log)
target_link_libraries(log ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/log.xml)
##################

##################
# Test that advertising a service multiple times fails
rospack_add_executable(service_adv_multiple service_adv_multiple.cpp)
rospack_add_gtest_build_flags(service_adv_multiple)
target_link_libraries(service_adv_multiple ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_adv_multiple.xml)
##################

##################
# Test that the second node to advertise a service "wins"
rospack_add_executable(service_adv_a service_adv_a.cpp)
rospack_add_gtest_build_flags(service_adv_a)
target_link_libraries(service_adv_a ros)

rospack_add_executable(service_wait_a_adv_b service_wait_a_adv_b.cpp)
rospack_add_gtest_build_flags(service_wait_a_adv_b)
target_link_libraries(service_wait_a_adv_b ros)

rospack_add_executable(service_call_expect_b service_call_expect_b.cpp)
rospack_add_gtest_build_flags(service_call_expect_b)
target_link_libraries(service_call_expect_b ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_multiple_providers.xml)
##################

##################
# Test command-line name remapping
rospack_add_executable(name_remapping name_remapping.cpp)
rospack_add_gtest_build_flags(name_remapping)
target_link_libraries(name_remapping ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/name_remapping.xml)
##################

##################
# Test namespaces
rospack_add_executable(namespaces namespaces.cpp)
rospack_add_gtest_build_flags(namespaces)
target_link_libraries(namespaces ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/namespaces.xml)
##################

##################
# Test params
rospack_add_executable(params params.cpp)
rospack_add_gtest_build_flags(params)
target_link_libraries(params ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/params.xml)
##################

##################
# Test getting information from the master
rospack_add_executable(get_master_information get_master_information.cpp)
rospack_add_gtest_build_flags(get_master_information)
target_link_libraries(get_master_information ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/get_master_information.xml)
##################

##################
# Test multiple subscriptions
rospack_add_executable(multiple_subscriptions multiple_subscriptions.cpp)
rospack_add_gtest_build_flags(multiple_subscriptions)
target_link_libraries(multiple_subscriptions ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_subscriptions.xml)
##################

rospack_add_executable(multiple_subscriptions_standalone multiple_subscriptions_standalone.cpp)
rospack_declare_test(multiple_subscriptions_standalone)
target_link_libraries(multiple_subscriptions_standalone ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_nothread.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large_nothread.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/subscribe_self.xml)

rospack_add_executable(check_master check_master.cpp)
rospack_add_gtest_build_flags(check_master)
target_link_libraries(check_master ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/check_master.xml)
