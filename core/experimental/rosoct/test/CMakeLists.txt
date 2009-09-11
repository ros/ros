rospack_add_executable(test/session_adv session_adv.cpp)
rospack_declare_test(test/session_adv)
target_link_libraries(test/session_adv ros)

rospack_add_executable(test/testcallservice testcallservice.cpp)
rospack_add_gtest_build_flags(test/testcallservice)
target_link_libraries(test/testcallservice ros)

rospack_add_executable(test/testadvservice testadvservice.cpp)
rospack_add_gtest_build_flags(test/testadvservice)
target_link_libraries(test/testadvservice ros)

rospack_add_executable(test/testcallsession testcallsession.cpp)
rospack_add_gtest_build_flags(test/testcallsession)
target_link_libraries(test/testcallsession ros)

rospack_add_executable(test/testpublish testpublish.cpp)
rospack_add_gtest_build_flags(test/testpublish)
target_link_libraries(test/testpublish ros)

rospack_add_executable(test/testsubscribe testsubscribe.cpp)
rospack_add_gtest_build_flags(test/testsubscribe)
target_link_libraries(test/testsubscribe ros)

set(_running_in_hudson $ENV{WORKSPACE})
# start the testing
if(NOT _running_in_hudson)
 rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testcallservice.xml)
 rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testadvservice.xml)
 rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testcallsession.xml)
 rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testpublish.xml)
 rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testsubscribe.xml)
endif(NOT _running_in_hudson)
