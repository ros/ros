cmake_minimum_required(VERSION 2.8)
project(ros_comm)
find_package(ROS COMPONENTS catkin genmsg cpp_common rostime
  roscpp_serialization roscpp_traits std_msgs
  )

foreach(subdir
    messages/rosgraph_msgs
    messages/std_srvs
    utilities/xmlrpcpp
    tools/rosconsole
    clients/cpp/roscpp
    tools
    clients/rospy
    utilities/message_filters
    )
  add_subdirectory(${subdir})
endforeach()

catkin_export_python()
