cmake_minimum_required(VERSION 2.8)
project(rosmaster)
find_package(catkin REQUIRED COMPONENTS rostest)

catkin_package()
catkin_package_export()

catkin_python_setup()

add_rostest(test/rosmaster.test)
add_rostest(test/paramserver.test)

catkin_add_nosetests(test)

install(FILES manifest.xml
  DESTINATION share/rosmaster)
