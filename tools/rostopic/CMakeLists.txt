cmake_minimum_required(VERSION 2.8)
project(rostopic)
find_package(catkin)

install(FILES manifest.xml 
  DESTINATION share/rostopic
  )
install(PROGRAMS scripts/rostopic
  DESTINATION share/rostopic/scripts
  )
