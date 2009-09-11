set(EXECUTABLE_OUTPUT_PATH $ENV{ROS_ROOT}/bin)
rospack_add_executable(global_rosplay rosplay.cpp time_publisher.cpp)
SET_TARGET_PROPERTIES(global_rosplay PROPERTIES OUTPUT_NAME rosplay)
