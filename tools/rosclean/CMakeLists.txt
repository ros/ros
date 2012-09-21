cmake_minimum_required(VERSION 2.8)
project(rosclean)
find_package(catkin REQUIRED)
catkin_package()
catkin_package_export()

catkin_python_setup()

catkin_add_nosetests(test)
