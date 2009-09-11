#build a separate executable into ROS_ROOT/bin

set(EXECUTABLE_OUTPUT_PATH $ENV{ROS_ROOT}/bin)
rospack_add_executable(global_rxloggerlevel rxloggerlevel.cpp)
target_link_libraries(global_rxloggerlevel rxtools)
SET_TARGET_PROPERTIES(global_rxloggerlevel PROPERTIES OUTPUT_NAME rxloggerlevel)
