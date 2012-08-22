include_directories(${GTEST_INCLUDE_DIRS})

link_directories(${GTEST_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}-handles EXCLUDE_FROM_ALL handles.cpp)
target_link_libraries(${PROJECT_NAME}-handles ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-timer_callbacks EXCLUDE_FROM_ALL timer_callbacks.cpp)
target_link_libraries(${PROJECT_NAME}-timer_callbacks ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-latching_publisher EXCLUDE_FROM_ALL latching_publisher.cpp)
target_link_libraries(${PROJECT_NAME}-latching_publisher ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-publish_n_fast EXCLUDE_FROM_ALL publish_n_fast.cpp)
target_link_libraries(${PROJECT_NAME}-publish_n_fast ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_self EXCLUDE_FROM_ALL subscribe_self.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_self ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-pub_sub EXCLUDE_FROM_ALL pub_sub.cpp)
target_link_libraries(${PROJECT_NAME}-pub_sub ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-sub_pub EXCLUDE_FROM_ALL sub_pub.cpp)
target_link_libraries(${PROJECT_NAME}-sub_pub ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-publish_empty EXCLUDE_FROM_ALL publish_empty.cpp)
target_link_libraries(${PROJECT_NAME}-publish_empty ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-publish_onsub EXCLUDE_FROM_ALL publish_onsub.cpp)
target_link_libraries(${PROJECT_NAME}-publish_onsub ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_n_fast EXCLUDE_FROM_ALL subscribe_n_fast.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_n_fast ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_empty EXCLUDE_FROM_ALL subscribe_empty.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_empty ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_resubscribe EXCLUDE_FROM_ALL subscribe_resubscribe.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_resubscribe ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_unsubscribe EXCLUDE_FROM_ALL subscribe_unsubscribe.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_unsubscribe ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-publish_unadvertise EXCLUDE_FROM_ALL publish_unadvertise.cpp)
target_link_libraries(${PROJECT_NAME}-publish_unadvertise ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_unsubscribe_repeatedly EXCLUDE_FROM_ALL subscribe_unsubscribe_repeatedly.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_unsubscribe_repeatedly ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-publish_constantly EXCLUDE_FROM_ALL publish_constantly.cpp)
target_link_libraries(${PROJECT_NAME}-publish_constantly ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-param_update_test EXCLUDE_FROM_ALL param_update_test.cpp)
target_link_libraries(${PROJECT_NAME}-param_update_test ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-real_time_test EXCLUDE_FROM_ALL real_time_test.cpp)
target_link_libraries(${PROJECT_NAME}-real_time_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-real_time_test rosgraph_msgs_gencpp)

add_executable(${PROJECT_NAME}-sim_time_test EXCLUDE_FROM_ALL sim_time_test.cpp)
target_link_libraries(${PROJECT_NAME}-sim_time_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-sim_time_test rosgraph_msgs_gencpp)

# Call a service
add_executable(${PROJECT_NAME}-service_adv EXCLUDE_FROM_ALL service_adv.cpp)
target_link_libraries(${PROJECT_NAME}-service_adv ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_adv_unadv EXCLUDE_FROM_ALL service_adv_unadv.cpp)
target_link_libraries(${PROJECT_NAME}-service_adv_unadv ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_call EXCLUDE_FROM_ALL service_call.cpp)
target_link_libraries(${PROJECT_NAME}-service_call ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_deadlock EXCLUDE_FROM_ALL service_deadlock.cpp)
target_link_libraries(${PROJECT_NAME}-service_deadlock ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-service_deadlock std_srvs_gencpp)

add_executable(${PROJECT_NAME}-service_call_repeatedly EXCLUDE_FROM_ALL service_call_repeatedly.cpp)
target_link_libraries(${PROJECT_NAME}-service_call_repeatedly ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_exception EXCLUDE_FROM_ALL service_exception.cpp)
target_link_libraries(${PROJECT_NAME}-service_exception ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-service_exception std_srvs_gencpp)


# Repeatedly call ros::init()
add_executable(${PROJECT_NAME}-multiple_init_fini EXCLUDE_FROM_ALL multiple_init_fini.cpp)
target_link_libraries(${PROJECT_NAME}-multiple_init_fini ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test node inspection functionality
add_executable(${PROJECT_NAME}-inspection EXCLUDE_FROM_ALL inspection.cpp)
target_link_libraries(${PROJECT_NAME}-inspection ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test that advertising a service multiple times fails
add_executable(${PROJECT_NAME}-service_adv_multiple EXCLUDE_FROM_ALL service_adv_multiple.cpp)
target_link_libraries(${PROJECT_NAME}-service_adv_multiple ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test that the second node to advertise a service "wins"
add_executable(${PROJECT_NAME}-service_adv_a EXCLUDE_FROM_ALL service_adv_a.cpp)
target_link_libraries(${PROJECT_NAME}-service_adv_a ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_wait_a_adv_b EXCLUDE_FROM_ALL service_wait_a_adv_b.cpp)
target_link_libraries(${PROJECT_NAME}-service_wait_a_adv_b ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-service_call_expect_b EXCLUDE_FROM_ALL service_call_expect_b.cpp)
target_link_libraries(${PROJECT_NAME}-service_call_expect_b ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test command-line name remapping
add_executable(${PROJECT_NAME}-name_remapping EXCLUDE_FROM_ALL name_remapping.cpp)
target_link_libraries(${PROJECT_NAME}-name_remapping ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}-name_remapping_with_ns EXCLUDE_FROM_ALL name_remapping_with_ns.cpp)
target_link_libraries(${PROJECT_NAME}-name_remapping_with_ns ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test namespaces
add_executable(${PROJECT_NAME}-namespaces EXCLUDE_FROM_ALL namespaces.cpp)
target_link_libraries(${PROJECT_NAME}-namespaces ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test params
add_executable(${PROJECT_NAME}-params EXCLUDE_FROM_ALL params.cpp)
target_link_libraries(${PROJECT_NAME}-params ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test getting information from the master
add_executable(${PROJECT_NAME}-get_master_information EXCLUDE_FROM_ALL get_master_information.cpp)
target_link_libraries(${PROJECT_NAME}-get_master_information ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

# Test multiple subscriptions
add_executable(${PROJECT_NAME}-multiple_subscriptions EXCLUDE_FROM_ALL multiple_subscriptions.cpp)
target_link_libraries(${PROJECT_NAME}-multiple_subscriptions ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-check_master EXCLUDE_FROM_ALL check_master.cpp)
target_link_libraries(${PROJECT_NAME}-check_master ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-wait_for_message EXCLUDE_FROM_ALL wait_for_message.cpp)
target_link_libraries(${PROJECT_NAME}-wait_for_message ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-loads_of_publishers EXCLUDE_FROM_ALL loads_of_publishers.cpp)
target_link_libraries(${PROJECT_NAME}-loads_of_publishers ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-incrementing_sequence EXCLUDE_FROM_ALL incrementing_sequence.cpp)
target_link_libraries(${PROJECT_NAME}-incrementing_sequence ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscription_callback_types EXCLUDE_FROM_ALL subscription_callback_types.cpp)
target_link_libraries(${PROJECT_NAME}-subscription_callback_types ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-subscription_callback_types std_msgs_gencpp)

add_executable(${PROJECT_NAME}-service_callback_types EXCLUDE_FROM_ALL service_callback_types.cpp)
target_link_libraries(${PROJECT_NAME}-service_callback_types ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-intraprocess_subscriptions EXCLUDE_FROM_ALL intraprocess_subscriptions.cpp)
target_link_libraries(${PROJECT_NAME}-intraprocess_subscriptions ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-nonconst_subscriptions EXCLUDE_FROM_ALL nonconst_subscriptions.cpp)
target_link_libraries(${PROJECT_NAME}-nonconst_subscriptions ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_retry_tcp EXCLUDE_FROM_ALL subscribe_retry_tcp.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_retry_tcp ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-subscribe_star EXCLUDE_FROM_ALL subscribe_star.cpp)
target_link_libraries(${PROJECT_NAME}-subscribe_star ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-subscribe_star std_srvs_gencpp)

add_executable(${PROJECT_NAME}-publisher_for_star_subscriber EXCLUDE_FROM_ALL publisher_for_star_subscriber.cpp)
target_link_libraries(${PROJECT_NAME}-publisher_for_star_subscriber ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-publisher_for_star_subscriber std_srvs_gencpp)

add_executable(${PROJECT_NAME}-parameter_validation EXCLUDE_FROM_ALL parameter_validation.cpp)
target_link_libraries(${PROJECT_NAME}-parameter_validation ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-param_locale_avoidance_test EXCLUDE_FROM_ALL param_locale_avoidance_test.cpp)
target_link_libraries(${PROJECT_NAME}-param_locale_avoidance_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-crashes_under_gprof EXCLUDE_FROM_ALL crashes_under_gprof.cpp)
target_link_libraries(${PROJECT_NAME}-crashes_under_gprof ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-test_remapping EXCLUDE_FROM_ALL test_remapping.cpp)
target_link_libraries(${PROJECT_NAME}-test_remapping ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-name_not_remappable EXCLUDE_FROM_ALL name_not_remappable.cpp)
target_link_libraries(${PROJECT_NAME}-name_not_remappable ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-test_ns_node_remapping EXCLUDE_FROM_ALL test_ns_node_remapping.cpp)
target_link_libraries(${PROJECT_NAME}-test_ns_node_remapping ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-test_search_param EXCLUDE_FROM_ALL test_search_param.cpp)
target_link_libraries(${PROJECT_NAME}-test_search_param ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}-left_right EXCLUDE_FROM_ALL left_right.cpp)
target_link_libraries(${PROJECT_NAME}-left_right ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-left_right std_msgs_gencpp)

add_executable(${PROJECT_NAME}-string_msg_expect EXCLUDE_FROM_ALL string_msg_expect.cpp)
target_link_libraries(${PROJECT_NAME}-string_msg_expect ${GTEST_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}-string_msg_expect std_msgs_gencpp)

if(TARGET tests)
  add_dependencies(tests
    ${PROJECT_NAME}-handles
    ${PROJECT_NAME}-timer_callbacks
    ${PROJECT_NAME}-latching_publisher
    ${PROJECT_NAME}-publish_n_fast
    ${PROJECT_NAME}-subscribe_self
    ${PROJECT_NAME}-pub_sub
    ${PROJECT_NAME}-sub_pub
    ${PROJECT_NAME}-publish_empty
    ${PROJECT_NAME}-publish_onsub
    ${PROJECT_NAME}-subscribe_n_fast
    ${PROJECT_NAME}-subscribe_empty
    ${PROJECT_NAME}-subscribe_resubscribe
    ${PROJECT_NAME}-subscribe_unsubscribe
    ${PROJECT_NAME}-publish_unadvertise
    ${PROJECT_NAME}-subscribe_unsubscribe_repeatedly
    ${PROJECT_NAME}-publish_constantly
    ${PROJECT_NAME}-param_update_test
    ${PROJECT_NAME}-real_time_test
    ${PROJECT_NAME}-sim_time_test
    ${PROJECT_NAME}-service_adv
    ${PROJECT_NAME}-service_adv_unadv
    ${PROJECT_NAME}-service_call
    ${PROJECT_NAME}-service_deadlock
    ${PROJECT_NAME}-service_exception
    ${PROJECT_NAME}-service_call_repeatedly
    ${PROJECT_NAME}-multiple_init_fini
    ${PROJECT_NAME}-inspection
    ${PROJECT_NAME}-service_adv_multiple
    ${PROJECT_NAME}-service_adv_a
    ${PROJECT_NAME}-service_wait_a_adv_b
    ${PROJECT_NAME}-service_call_expect_b
    ${PROJECT_NAME}-name_remapping
    ${PROJECT_NAME}-name_remapping_with_ns
    ${PROJECT_NAME}-namespaces
    ${PROJECT_NAME}-params
    ${PROJECT_NAME}-get_master_information
    ${PROJECT_NAME}-multiple_subscriptions
    ${PROJECT_NAME}-check_master
    ${PROJECT_NAME}-wait_for_message
    ${PROJECT_NAME}-loads_of_publishers
    ${PROJECT_NAME}-incrementing_sequence
    ${PROJECT_NAME}-subscription_callback_types
    ${PROJECT_NAME}-service_callback_types
    ${PROJECT_NAME}-intraprocess_subscriptions
    ${PROJECT_NAME}-nonconst_subscriptions
    ${PROJECT_NAME}-subscribe_retry_tcp
    ${PROJECT_NAME}-subscribe_star
    ${PROJECT_NAME}-publisher_for_star_subscriber
    ${PROJECT_NAME}-parameter_validation
    ${PROJECT_NAME}-param_locale_avoidance_test
    ${PROJECT_NAME}-crashes_under_gprof
    ${PROJECT_NAME}-test_remapping
    ${PROJECT_NAME}-name_not_remappable
    ${PROJECT_NAME}-test_ns_node_remapping
    ${PROJECT_NAME}-test_search_param
    ${PROJECT_NAME}-left_right
    ${PROJECT_NAME}-string_msg_expect
  )
endif()

add_dependencies(${PROJECT_NAME}-handles ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-timer_callbacks ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-latching_publisher ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publish_n_fast ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_self ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-pub_sub ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-sub_pub ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publish_empty ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publish_onsub ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_n_fast ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_empty ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_resubscribe ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_unsubscribe ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publish_unadvertise ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_unsubscribe_repeatedly ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publish_constantly ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-param_update_test ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-real_time_test ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-sim_time_test ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_adv ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_adv_unadv ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_call ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_deadlock ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_exception ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_call_repeatedly ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-multiple_init_fini ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-inspection ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_adv_multiple ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_adv_a ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_wait_a_adv_b ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_call_expect_b ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-name_remapping ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-name_remapping_with_ns ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-namespaces ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-params ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-get_master_information ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-multiple_subscriptions ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-check_master ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-wait_for_message ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-loads_of_publishers ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-incrementing_sequence ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscription_callback_types ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-service_callback_types ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-intraprocess_subscriptions ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-nonconst_subscriptions ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_retry_tcp ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-subscribe_star ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-publisher_for_star_subscriber ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-parameter_validation ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-param_locale_avoidance_test ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-crashes_under_gprof ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-test_remapping ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-name_not_remappable ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-test_ns_node_remapping ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-test_search_param ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-left_right ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}-string_msg_expect ${PROJECT_NAME}_gencpp)
