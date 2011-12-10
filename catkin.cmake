cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin)

foreach(subdir
    core/roslib
    tools/rosbash
    tools/rosclean
    )
  add_subdirectory(${subdir})
endforeach()

catkin_package(ros)
