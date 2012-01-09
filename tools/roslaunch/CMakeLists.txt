cmake_minimum_required(VERSION 2.8)
project(roslaunch)
find_package(catkin)

add_rostest(test/rostests/roslaunch.test)
add_rostest(test/rostests/env.test)
add_rostest(test/rostests/params_basic.test)

# need to actually implement logic to switch to etc finding
message("TODO: install roscore.xml to /etc/ros")
#install(FILES roscore.xml
#  DESTINATION etc/ros
#  )
# delete this once above is in placae
install(FILES roscore.xml
  DESTINATION share/roslaunch
  )

# install example launch files
install(DIRECTORY resources
  DESTINATION share/roslaunch
  )
