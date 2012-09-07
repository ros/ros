find_package(catkin REQUIRED COMPONENTS roscpp rosconsole xmlrpcpp)
include_directories(${catkin_INCLUDE_DIRS})

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

install(FILES rosmsg/manifest.xml
  DESTINATION share/rosmsg)

install(FILES rosservice/manifest.xml
  DESTINATION share/rosservice)
install(PROGRAMS rosservice/scripts/rosservice
  DESTINATION lib/rosservice)

catkin_add_nosetests(rosmsg/test)
