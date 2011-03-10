include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

rosbuild_add_library(${PROJECT_NAME} src/serialization.cpp)