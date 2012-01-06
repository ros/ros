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
    utilities/message_filters

    # for tests
    utilities/roswtf
    test/test_ros
    test/test_rospy
    test/test_rosservice
    )
  add_subdirectory(${subdir})
endforeach()

catkin_export_python()

add_nosetests(tools/rosmaster/test)
add_nosetests(tools/rosmsg/test)
add_nosetests(tools/rosparam/test)
add_nosetests(tools/rosgraph/test)
add_nosetests(tools/roslaunch/test/unit)
add_nosetests(utilities/roswtf/test)
