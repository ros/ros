cmake_minimum_required(VERSION 2.8)
project(rosnode)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

add_rostest(test/rosnode.test)
