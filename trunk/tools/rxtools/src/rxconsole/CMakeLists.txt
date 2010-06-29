#build a separate executable into ROS_ROOT/bin

set(EXECUTABLE_OUTPUT_PATH $ENV{ROS_ROOT}/bin)
rosbuild_add_executable(global_rxconsole rxconsole.cpp)
target_link_libraries(global_rxconsole rxtools)
SET_TARGET_PROPERTIES(global_rxconsole PROPERTIES OUTPUT_NAME rxconsole)
