include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

#
#  Make a buildspace rosconsole.config if we've got one handy
#
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/ros/config/rosconsole.config)
  configure_file(${CMAKE_SOURCE_DIR}/ros/config/rosconsole.config
    ${CMAKE_BINARY_DIR}/rosconsole.config
    @ONLY
    )
endif()

include_directories(include/rosconsole)
rosbuild_add_boost_directories()
rosbuild_add_library(${PROJECT_NAME} src/rosconsole/rosconsole.cpp)
rosbuild_link_boost(${PROJECT_NAME} thread regex)
rosbuild_add_executable(example examples/example.cpp)
target_link_libraries(example ${PROJECT_NAME})
# static libraries all need to be called for mingw. 
if(MINGW)
rosbuild_link_boost(example thread regex)
endif()
set_target_properties(example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)
rosbuild_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest ${PROJECT_NAME})
if(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
rosbuild_add_gtest(test/assertion_test test/assertion_test.cpp)
target_link_libraries(test/assertion_test ${PROJECT_NAME})
endif(${CMAKE_SYSTEM_NAME} STREQUAL Linux)
rosbuild_add_gtest(test/thread_test test/thread_test.cpp)
target_link_libraries(test/thread_test ${PROJECT_NAME})
#rosbuild_add_executable(speed_test test/speed_test.cpp)
#target_link_libraries(speed_test ${PROJECT_NAME})
#set_target_properties(speed_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

