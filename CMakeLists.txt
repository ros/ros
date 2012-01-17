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

add_nosetests(test/test_roslib_comm/test)
add_nosetests(tools/rosmaster/test)
add_nosetests(tools/rosmsg/test)
add_nosetests(tools/rosparam/test)
add_nosetests(tools/rosgraph/test)
add_nosetests(tools/roslaunch/test/unit)
add_nosetests(utilities/roswtf/test)

# install legacy infrastructure needed by rosbuild
# rospy
install(FILES clients/rospy/rosbuild/rospy.cmake
        DESTINATION share/rospy/rosbuild)
install(PROGRAMS 
        clients/rospy/rosbuild/scripts/genmsg_py.py
        clients/rospy/rosbuild/scripts/gensrv_py.py
        DESTINATION share/rospy/rosbuild/scripts)
install(FILES 
        clients/rospy/rosbuild/scripts/genutil.py
        DESTINATION share/rospy/rosbuild/scripts)
# roslisp
install(FILES clients/roslisp/rosbuild/roslisp.cmake
        DESTINATION share/roslisp/rosbuild)
install(PROGRAMS 
        clients/roslisp/rosbuild/scripts/genmsg_lisp.py
        DESTINATION share/roslisp/rosbuild/scripts)
install(PROGRAMS 
        clients/cpp/roscpp/rosbuild/scripts/genmsg_cpp.py
        clients/cpp/roscpp/rosbuild/scripts/gensrv_cpp.py
        DESTINATION share/roscpp/rosbuild/scripts)

# install all manifests
install_matching_to_share(manifest.xml)

install(FILES stack.xml rosdep.yaml
        DESTINATION share/ros_comm)

