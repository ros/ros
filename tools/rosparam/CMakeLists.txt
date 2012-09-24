cmake_minimum_required(VERSION 2.8)
project(rosparam)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

catkin_python_setup()

add_rostest(test/rosparam.test)

catkin_add_nosetests(test)
