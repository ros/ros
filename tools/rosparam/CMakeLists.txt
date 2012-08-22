cmake_minimum_required(VERSION 2.8)
project(rosparam)
find_package(catkin REQUIRED COMPONENTS rostest)

add_rostest(test/rosparam.test)
