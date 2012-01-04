find_package(ROS REQUIRED COMPONENTS roscpp rosconsole XmlRpc)
include_directories(${ROS_INCLUDE_DIRS})

foreach(subdir
    # rosconsole can NOT go here since roscpp depends on it.
    # It is add_subdirred from one level up.
    # rosconsole
    topic_tools
    rosbag
    rosgraph
    rosout
    rosservice
    rostopic
    )
  add_subdirectory(${subdir})
endforeach()
