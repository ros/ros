if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)
endif()
#set(ROS_BUILD_TYPE Debug)
add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL)

