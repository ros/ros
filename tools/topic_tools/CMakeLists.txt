project(topic_tools)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

include_directories(include)
find_package(ROS COMPONENTS roscpp rosconsole XmlRpc)
include_directories(${ROS_INCLUDE_DIRS})

rosbuild_add_library(topic_tools src/shape_shifter.cpp src/parse.cpp)
target_link_libraries(topic_tools ${ROS_LIBRARIES})

rosbuild_add_executable(switch_mux src/switch_mux.cpp)
target_link_libraries(switch_mux topic_tools ${ROS_LIBRARIES})

rosbuild_add_executable(mux src/mux.cpp)
target_link_libraries(mux topic_tools ${ROS_LIBRARIES})
add_dependencies(topic_tools topic_tools_gencpp)

rosbuild_add_executable(relay src/relay.cpp)
target_link_libraries(relay topic_tools ${ROS_LIBRARIES})

rosbuild_add_executable(drop src/drop.cpp)
target_link_libraries(drop topic_tools ${ROS_LIBRARIES})

#rosbuild_add_executable(demux demux.cpp)
#target_link_libraries(demux topic_tools)

rosbuild_add_executable(throttle src/throttle.cpp)
target_link_libraries(throttle topic_tools ${ROS_LIBRARIES})

add_service_files(DIRECTORY srv
  FILES MuxAdd.srv MuxDelete.srv MuxList.srv MuxSelect.srv
  )

generate_messages(DEPENDENCIES std_msgs)

install_cmake_infrastructure(topic_tools
  VERSION 0.0.0
  LIBRARIES topic_tools
  INCLUDE_DIRS include
  )
