cmake_minimum_required(VERSION 2.8)
project(ros_comm)
find_package(ROS COMPONENTS catkin genmsg cpp_common rostime roscpp_serialization roscpp_traits)

foreach(subdir
    messages/rosgraph_msgs
    messages/std_srvs
    utilities/xmlrpcpp
    tools/rosconsole
    clients/cpp/roscpp
    clients/rospy
    utilities/message_filters
    tools
    )
  add_subdirectory(${subdir})
endforeach()

catkin_package(ros_comm)
