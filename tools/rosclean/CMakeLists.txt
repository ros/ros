cmake_minimum_required(VERSION 2.8.3)
project(rosclean)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
