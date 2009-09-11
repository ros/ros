rospack_add_executable(test/session_adv session_adv.cpp)
rospack_declare_test(test/session_adv)

rospack_add_executable(test/session_call_repeatedly session_call_repeatedly.cpp)
rospack_declare_test(test/session_call_repeatedly)

rospack_add_executable(test/session_adv_test session_adv_test.cpp)
rospack_add_gtest_build_flags(test/session_adv_test)

rospack_add_executable(test/session_call session_call.cpp)
rospack_add_gtest_build_flags(test/session_call)

##################
# session testing
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/session_adv.xml)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/session_call.xml)
