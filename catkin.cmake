cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin)
find_package(Boost COMPONENTS thread)

foreach(subdir
    core/roslib
    tools/rosbash
    tools/rosclean
    )
  add_subdirectory(${subdir})
endforeach()

catkin_package(ros)

install(PROGRAMS
  bin/rosrun
  bin/rostopic
  bin/rxconsole
  bin/rosbag
  DESTINATION bin
  )
