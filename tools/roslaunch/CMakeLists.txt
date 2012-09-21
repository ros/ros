cmake_minimum_required(VERSION 2.8)
project(roslaunch)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_package()
catkin_package_export()

catkin_python_setup()

install(FILES resources/roscore.xml
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)

# install example launch files
install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_rostest(test/rostests/roslaunch.test)
add_rostest(test/rostests/env.test)
add_rostest(test/rostests/params_basic.test)

catkin_add_nosetests(test/unit)
