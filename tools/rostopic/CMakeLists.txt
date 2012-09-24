cmake_minimum_required(VERSION 2.8)
project(rostopic)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()

catkin_python_setup()

add_rostest(test/rostopic.test)

install(PROGRAMS scripts/rostopic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
