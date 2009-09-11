rospack_add_gtest(test/test_version test_version.cpp)
target_link_libraries(test/test_version)

rospack_add_gtest(test/test_poll_set test_poll_set.cpp)
target_link_libraries(test/test_poll_set ros)

rospack_add_gtest(test/test_transport_tcp test_transport_tcp.cpp)
target_link_libraries(test/test_transport_tcp ros)
