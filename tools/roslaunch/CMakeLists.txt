cmake_minimum_required(VERSION 2.8)
project(roslaunch)
find_package(catkin)

message("TODO: enable roslaunch rostests")
#add_rostest(test/rostest/roslaunch.test)
#add_rostest(test/rostest/env.test)
#add_rostest(test/rostest/params_basic.test)

# need to actually implement logic to switch to etc finding
message("TODO: install roscore.xml to /etc/ros")
#install(FILES roscore.xml
#  DESTINATION etc/ros
#  )
# delete this once above is in placae
install(FILES roscore.xml
  DESTINATION share/roslaunch
  )
