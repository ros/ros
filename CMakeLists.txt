cmake_minimum_required(VERSION 2.8)
project(ros_comm)
find_package(ROS COMPONENTS catkin genmsg cpp_common rostime
  roscpp_serialization roscpp_traits std_msgs
  )

# Must export python before recursing into subdirs, so that rostest is
# linked into build/bin before anybody calls add_rostest().
catkin_export_python()

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
    test/test_roslib_comm
    test/test_rospy
    test/test_rosservice
    )
  add_subdirectory(${subdir})
endforeach()

add_nosetests(tools/rosmaster/test)
add_nosetests(tools/rosmsg/test)
add_nosetests(tools/rosparam/test)
add_nosetests(tools/rosgraph/test)
add_nosetests(tools/roslaunch/test/unit)
add_nosetests(utilities/roswtf/test)

install(FILES clients/rospy/cmake/rosbuild-rospy.cmake
        DESTINATION share/rospy/cmake)
install(FILES clients/roslisp/cmake/rosbuild-roslisp.cmake
        DESTINATION share/roslisp/cmake)

# install all manifests
install_matching_to_share(manifest.xml)

