cmake_minimum_required(VERSION 2.8)
project(rospy)
find_package(catkin REQUIRED)
catkin_project(${PROJECT_NAME})

# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/rospy.cmake
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/rosbuild)
install(PROGRAMS
  rosbuild/scripts/genmsg_py.py
  rosbuild/scripts/gensrv_py.py
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/rosbuild/scripts)
install(FILES rosbuild/scripts/genutil.py
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/rosbuild/scripts)
