cmake_minimum_required(VERSION 2.8)
project(rospy)
find_package(catkin)

catkin_project(rospy
  PYTHONPATH src
  )

enable_python(rospy)

