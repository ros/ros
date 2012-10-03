cmake_minimum_required(VERSION 2.8.3)
project(roscreate)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(DIRECTORY templates/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/templates
  PATTERN ".svn" EXCLUDE)

catkin_add_nosetests(test)
