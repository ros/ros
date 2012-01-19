cmake_minimum_required(VERSION 2.8)
project(rosnode)
find_package(ROS COMPONENTS catkin rostest)

add_rostest(test/rosnode.test)
