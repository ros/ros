cmake_minimum_required(VERSION 2.8)
project(roslaunch)
find_package(ROS COMPONENTS catkin rostest)

add_rostest(test/rostests/roslaunch.test)
add_rostest(test/rostests/env.test)
add_rostest(test/rostests/params_basic.test)

install(FILES resources/roscore.xml
  DESTINATION etc/ros
  )

# install example launch files
install(DIRECTORY resources
  DESTINATION share/roslaunch
  PATTERN ".svn" EXCLUDE
  )
