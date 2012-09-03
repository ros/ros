cmake_minimum_required(VERSION 2.8)
project(rostopic)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_project(rostopic)

add_rostest(test/rostopic.test)

install(PROGRAMS scripts/rostopic
  DESTINATION ${PROJECT_LIBEXEC_DESTINATION})
