cmake_minimum_required(VERSION 2.8)
project(ros_comm)
find_package(catkin REQUIRED COMPONENTS genmsg cpp_common rostime
  roscpp_serialization roscpp_traits std_msgs rosunit)
catkin_stack()
catkin_project(ros_comm)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Must call catkin_python_setup() before recursing into subdirs
# so that rostest is available in build-space
# before anybody calls add_rostest().
catkin_python_setup()

foreach(subdir
    tools/rostest
    messages/rosgraph_msgs
    messages/std_srvs
    utilities/xmlrpcpp
    tools/rosconsole
    clients/cpp/roscpp
    clients/rospy
    tools
    utilities/message_filters

    # for tests
    utilities/roswtf
    test/test_ros
    test/test_roslib_comm
    test/test_rospy
    test/test_rosservice
    test/test_roscpp
    test/test_roscpp_serialization
    test/test_roscpp_serialization_perf
    test/test_rostime
    test/perf_roscpp
    test/test_crosspackage
    test/test_rosbag
    )
  add_subdirectory(${subdir})
endforeach()

catkin_add_nosetests(tools/rosmsg/test)
