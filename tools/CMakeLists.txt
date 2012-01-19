find_package(ROS REQUIRED COMPONENTS roscpp rosconsole xmlrpcpp)
include_directories(${ROS_INCLUDE_DIRS})

foreach(subdir
    # rosconsole can NOT go here since roscpp depends on it.
    # It is add_subdirred from one level up.
    # rosconsole
    topic_tools
    rosbag
    rosgraph
    roslaunch
    rosout
    rostopic

    # for test integration
    rosmaster
    rosnode
    rosparam
    )
  add_subdirectory(${subdir})
endforeach()

install(PROGRAMS rosservice/scripts/rosservice
  DESTINATION share/rosservice/scripts
  )
