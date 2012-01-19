cmake_minimum_required(VERSION 2.8)
project(rostopic)
find_package(ROS COMPONENTS catkin rostest)

add_rostest(test/rostopic.test)

install(PROGRAMS scripts/rostopic
  DESTINATION share/rostopic/scripts
  )
