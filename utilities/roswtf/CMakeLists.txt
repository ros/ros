cmake_minimum_required(VERSION 2.8)
project(roswtf)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_project(${PROJECT_NAME})

add_rostest(test/roswtf.test)

catkin_add_nosetests(test)
