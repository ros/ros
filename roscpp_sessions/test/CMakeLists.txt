rosbuild_add_executable(test/session_adv session_adv.cpp)
rosbuild_declare_test(test/session_adv)

rosbuild_add_executable(test/session_call_repeatedly session_call_repeatedly.cpp)
rosbuild_declare_test(test/session_call_repeatedly)

rosbuild_add_executable(test/session_adv_test session_adv_test.cpp)
rosbuild_add_gtest_build_flags(test/session_adv_test)

rosbuild_add_executable(test/session_call session_call.cpp)
rosbuild_add_gtest_build_flags(test/session_call)

##################
# session testing
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/session_adv.xml)
rosbuild_add_rostest_future(${CMAKE_CURRENT_SOURCE_DIR}/session_call.xml)
