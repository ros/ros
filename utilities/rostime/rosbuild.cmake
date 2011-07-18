include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

rosbuild_add_boost_directories()

rosbuild_add_library(${PROJECT_NAME} 
  src/time.cpp src/rate.cpp src/duration.cpp)

rosbuild_link_boost(${PROJECT_NAME} date_time thread) 

if(NOT (APPLE OR WIN32 OR MINGW))
  target_link_libraries(${PROJECT_NAME} rt)
endif()

