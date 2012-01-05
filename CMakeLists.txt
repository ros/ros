cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin)
find_package(Boost COMPONENTS thread)

foreach(subdir
    core/roslib
    tools/rosbash
    )
  add_subdirectory(${subdir})
endforeach()


install(PROGRAMS
  bin/rosrun
  bin/rxconsole
  DESTINATION bin
  )

catkin_export_python()
