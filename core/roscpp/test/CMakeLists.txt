rospack_add_executable(test/publish_n_fast publish_n_fast.cpp)
rospack_declare_test(test/publish_n_fast)
target_link_libraries(test/publish_n_fast ros)

rospack_add_executable(test/subscribe_self subscribe_self.cpp)
rospack_add_gtest_build_flags(test/subscribe_self)
target_link_libraries(test/subscribe_self ros)

rospack_add_executable(test/pub_sub pub_sub.cpp)
rospack_declare_test(test/pub_sub)
target_link_libraries(test/pub_sub ros)

rospack_add_executable(test/sub_pub sub_pub.cpp)
rospack_add_gtest_build_flags(test/sub_pub)
target_link_libraries(test/sub_pub ros)

rospack_add_executable(test/publish_empty publish_empty.cpp)
rospack_declare_test(test/publish_empty)
target_link_libraries(test/publish_empty ros)

rospack_add_executable(test/publish_onsub publish_onsub.cpp)
rospack_declare_test(test/publish_onsub)
target_link_libraries(test/publish_onsub ros)

rospack_add_executable(test/subscribe_n_fast subscribe_n_fast.cpp)
rospack_add_gtest_build_flags(test/subscribe_n_fast)
target_link_libraries(test/subscribe_n_fast ros)

rospack_add_executable(test/subscribe_empty subscribe_empty.cpp)
rospack_add_gtest_build_flags(test/subscribe_empty)
target_link_libraries(test/subscribe_empty ros)

rospack_add_executable(test/subscribe_resubscribe subscribe_resubscribe.cpp)
rospack_add_gtest_build_flags(test/subscribe_resubscribe)
target_link_libraries(test/subscribe_resubscribe ros)

rospack_add_executable(test/subscribe_unsubscribe subscribe_unsubscribe.cpp)
rospack_add_gtest_build_flags(test/subscribe_unsubscribe)
target_link_libraries(test/subscribe_unsubscribe ros)

rospack_add_executable(test/publish_unadvertise publish_unadvertise.cpp)
rospack_add_gtest_build_flags(test/publish_unadvertise)
target_link_libraries(test/publish_unadvertise ros)

rospack_add_executable(test/subscribe_unsubscribe_repeatedly subscribe_unsubscribe_repeatedly.cpp)
rospack_declare_test(test/subscribe_unsubscribe_repeatedly)
target_link_libraries(test/subscribe_unsubscribe_repeatedly ros)

rospack_add_executable(test/publish_constantly publish_constantly.cpp)
rospack_declare_test(test/publish_constantly)
target_link_libraries(test/publish_constantly ros)

rospack_add_executable(test/shutdown_master shutdown_master.cpp)
rospack_declare_test(test/shutdown_master)
target_link_libraries(test/shutdown_master ros)

rospack_add_executable(test/param_update_test param_update_test.cpp)
rospack_declare_test(test/param_update_test)
target_link_libraries(test/param_update_test ros)
 
##################
rospack_add_executable(test/real_time_test real_time_test.cpp)
rospack_declare_test(test/real_time_test)
rospack_add_gtest_build_flags(test/real_time_test)
target_link_libraries(test/real_time_test ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/real_time_test.xml)
##################

##################
rospack_add_executable(test/sim_time_test sim_time_test.cpp)
rospack_declare_test(test/sim_time_test)
rospack_add_gtest_build_flags(test/sim_time_test)
target_link_libraries(test/sim_time_test ros)
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
rospack_add_executable(test/service_adv service_adv.cpp)
rospack_declare_test(test/service_adv)
target_link_libraries(test/service_adv ros)

rospack_add_executable(test/service_adv_unadv service_adv_unadv.cpp)
rospack_declare_test(test/service_adv_unadv)
rospack_add_gtest_build_flags(test/service_adv_unadv)
target_link_libraries(test/service_adv_unadv ros)

rospack_add_executable(test/service_call service_call.cpp)
rospack_add_gtest_build_flags(test/service_call)
target_link_libraries(test/service_call ros)

rospack_add_executable(test/service_call_repeatedly service_call_repeatedly.cpp)
rospack_declare_test(test/service_call_repeatedly)
target_link_libraries(test/service_call_repeatedly ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_call_unadv_nothread.xml)
##################

##################
# Repeatedly call ros::init() and ros::fini()
rospack_add_executable(test/multiple_init_fini multiple_init_fini.cpp)
rospack_add_gtest_build_flags(test/multiple_init_fini)
target_link_libraries(test/multiple_init_fini ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_init_fini.xml)
##################

##################
# Test node inspection functionality
rospack_add_executable(test/inspection inspection.cpp)
rospack_add_gtest_build_flags(test/inspection)
target_link_libraries(test/inspection ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/inspection.xml)
##################

##################
# Test logging functionality
rospack_add_executable(test/log log.cpp)
rospack_add_gtest_build_flags(test/log)
target_link_libraries(test/log ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/log.xml)
##################

##################
# Test that advertising a service multiple times fails
rospack_add_executable(test/service_adv_multiple service_adv_multiple.cpp)
rospack_add_gtest_build_flags(test/service_adv_multiple)
target_link_libraries(test/service_adv_multiple ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_adv_multiple.xml)
##################

##################
# Test that the second node to advertise a service "wins"
rospack_add_executable(test/service_adv_a service_adv_a.cpp)
rospack_add_gtest_build_flags(test/service_adv_a)
target_link_libraries(test/service_adv_a ros)

rospack_add_executable(test/service_wait_a_adv_b service_wait_a_adv_b.cpp)
rospack_add_gtest_build_flags(test/service_wait_a_adv_b)
target_link_libraries(test/service_wait_a_adv_b ros)

rospack_add_executable(test/service_call_expect_b service_call_expect_b.cpp)
rospack_add_gtest_build_flags(test/service_call_expect_b)
target_link_libraries(test/service_call_expect_b ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/service_multiple_providers.xml)
##################

##################
# Test command-line name remapping
rospack_add_executable(test/name_remapping name_remapping.cpp)
rospack_add_gtest_build_flags(test/name_remapping)
target_link_libraries(test/name_remapping ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/name_remapping.xml)
##################

##################
# Test namespaces
rospack_add_executable(test/namespaces namespaces.cpp)
rospack_add_gtest_build_flags(test/namespaces)
target_link_libraries(test/namespaces ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/namespaces.xml)
##################

##################
# Test params
rospack_add_executable(test/params params.cpp)
rospack_add_gtest_build_flags(test/params)
target_link_libraries(test/params ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/params.xml)
##################

##################
# Test getting information from the master
rospack_add_executable(test/get_master_information get_master_information.cpp)
rospack_add_gtest_build_flags(test/get_master_information)
target_link_libraries(test/get_master_information ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/get_master_information.xml)
##################

##################
# Test multiple subscriptions
rospack_add_executable(test/multiple_subscriptions multiple_subscriptions.cpp)
rospack_add_gtest_build_flags(test/multiple_subscriptions)
target_link_libraries(test/multiple_subscriptions ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/multiple_subscriptions.xml)
##################

rospack_add_executable(test/multiple_subscriptions_standalone multiple_subscriptions_standalone.cpp)
rospack_declare_test(multiple_subscriptions_standalone)
target_link_libraries(test/multiple_subscriptions_standalone ros)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_nothread.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/pingpong_large_nothread.xml)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/subscribe_self.xml)

rospack_add_executable(test/check_master check_master.cpp)
rospack_add_gtest_build_flags(test/check_master)
target_link_libraries(test/check_master ros)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/check_master.xml)
