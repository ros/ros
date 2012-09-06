cmake_minimum_required(VERSION 2.8)
project(roslaunch)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_project(roslaunch)

add_rostest(test/rostests/roslaunch.test)
add_rostest(test/rostests/env.test)
add_rostest(test/rostests/params_basic.test)

install(FILES resources/roscore.xml
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)

# install example launch files
install(DIRECTORY resources
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION})
