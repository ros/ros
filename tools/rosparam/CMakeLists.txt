cmake_minimum_required(VERSION 2.8)
project(rosparam)
find_package(ROS COMPONENTS catkin rostest)

add_rostest(test/rosparam.test)
