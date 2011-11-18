cmake_minimum_required(VERSION 2.8)
project(rospy)
find_package(catkin)

install_cmake_infrastructure(rospy
  PYTHONPATH src
  )

enable_python(rospy)

