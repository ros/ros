cmake_minimum_required(VERSION 2.8)
project(rosmake)
find_package(catkin REQUIRED)
catkin_project(${PROJECT_NAME})

catkin_add_nosetests(test)
