set(EXECUTABLE_OUTPUT_PATH $ENV{ROS_ROOT}/bin)
rospack_add_executable(global_rosrecord rosrecord.cpp)
SET_TARGET_PROPERTIES(global_rosrecord PROPERTIES OUTPUT_NAME rosrecord)