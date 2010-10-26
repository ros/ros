rosbuild_add_gtest(test/test_version test_version.cpp)
target_link_libraries(test/test_version)

rosbuild_add_gtest(test/test_header test_header.cpp)
target_link_libraries(test/test_header ros)

rosbuild_add_gtest(test/test_poll_set test_poll_set.cpp)
target_link_libraries(test/test_poll_set ros)

rosbuild_add_gtest(test/test_transport_tcp test_transport_tcp.cpp)
target_link_libraries(test/test_transport_tcp ros)

rosbuild_add_gtest(test/test_subscription_queue test_subscription_queue.cpp)
target_link_libraries(test/test_subscription_queue ros)

rosbuild_add_gtest(test/test_callback_queue test_callback_queue.cpp)
target_link_libraries(test/test_callback_queue ros)

rosbuild_add_gtest(test/test_names test_names.cpp)
target_link_libraries(test/test_names ros)

rosbuild_add_gtest(test/test_args test_args.cpp)
target_link_libraries(test/test_args ros)
