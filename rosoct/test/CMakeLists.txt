rosbuild_add_executable(test/session_adv session_adv.cpp)
rosbuild_declare_test(test/session_adv)
target_link_libraries(test/session_adv ros)

rosbuild_add_executable(test/testcallservice testcallservice.cpp)
rosbuild_add_gtest_build_flags(test/testcallservice)
target_link_libraries(test/testcallservice ros)

rosbuild_add_executable(test/testadvservice testadvservice.cpp)
rosbuild_add_gtest_build_flags(test/testadvservice)
target_link_libraries(test/testadvservice ros)

rosbuild_add_executable(test/testcallsession testcallsession.cpp)
rosbuild_add_gtest_build_flags(test/testcallsession)
target_link_libraries(test/testcallsession ros)

rosbuild_add_executable(test/testpublish testpublish.cpp)
rosbuild_add_gtest_build_flags(test/testpublish)
target_link_libraries(test/testpublish ros)

rosbuild_add_executable(test/testsubscribe testsubscribe.cpp)
rosbuild_add_gtest_build_flags(test/testsubscribe)
target_link_libraries(test/testsubscribe ros)

set(_running_in_hudson $ENV{WORKSPACE})
# start the testing
if(NOT _running_in_hudson)
 rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testcallservice.xml)
 rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testadvservice.xml)
 rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testcallsession.xml)
 rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testpublish.xml)
 rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/testsubscribe.xml)
endif(NOT _running_in_hudson)
