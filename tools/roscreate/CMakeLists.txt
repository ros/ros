cmake_minimum_required(VERSION 2.8)
project(roscreate)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_add_nosetests(test)
