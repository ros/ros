cmake_minimum_required(VERSION 2.8)
project(roswtf)
find_package(catkin COMPONENTS rostest)

add_rostest(test/roswtf.test)
