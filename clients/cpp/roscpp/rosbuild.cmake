include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL)

