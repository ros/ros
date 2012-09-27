cmake_minimum_required(VERSION 2.8)
project(rosbuild)
find_package(catkin REQUIRED)
catkin_package()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
install(FILES
  FindPkgConfig.cmake
  private.cmake
  ProcessorCount.cmake
  public.cmake
  rosbuild.cmake
  rosconfig.cmake
  rostoolchain.cmake
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild)
install(FILES
  tests/count_cores.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/tests)
install(PROGRAMS
  bin/check_same_directories.py
  bin/download_checkmd5.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/bin)
