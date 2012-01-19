cmake_minimum_required(VERSION 2.8)
project(rosmaster)
find_package(ROS COMPONENTS catkin rostest)

add_rostest(test/rosmaster.test)
add_rostest(test/paramserver.test)
