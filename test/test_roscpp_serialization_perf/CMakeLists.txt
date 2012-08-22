project(test_roscpp_serialization_perf)
catkin_project(test_roscpp_serialization_perf)
find_package(catkin COMPONENTS cpp_common rostime roscpp_serialization roscpp_traits)
include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  DIRECTORY msg
  FILES
  ChannelFloat32.msg
  Point32.msg
  PointCloud.msg
)
generate_messages(DEPENDENCIES std_msgs)

add_executable(${PROJECT_NAME}-pointcloud_serdes EXCLUDE_FROM_ALL pointcloud_serdes.cpp)
target_link_libraries(${PROJECT_NAME}-pointcloud_serdes ${catkin_LIBRARIES})
if(TARGET tests)
  add_dependencies(tests ${PROJECT_NAME}-pointcloud_serdes) 
endif()
add_dependencies(${PROJECT_NAME}-pointcloud_serdes ${PROJECT_NAME}_gencpp) 
#rosbuild_add_compile_flags(pointcloud_serdes "-O3 -funroll-loops")
#rosbuild_add_compile_flags(pointcloud_serdes "-march=prescott")

