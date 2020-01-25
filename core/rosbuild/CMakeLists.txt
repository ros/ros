cmake_minimum_required(VERSION 3.0.2)
project(rosbuild)
find_package(catkin REQUIRED)
catkin_package()

if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(10.rosbuild SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
else()
  catkin_add_env_hooks(10.rosbuild SHELLS bat DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
endif()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
install(FILES
  core/rosbuild/FindPkgConfig.cmake
  core/rosbuild/private.cmake
  core/rosbuild/ProcessorCount.cmake
  core/rosbuild/public.cmake
  core/rosbuild/rosbuild.cmake
  core/rosbuild/rosconfig.cmake
  core/rosbuild/rostoolchain.cmake
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild)
catkin_install_python(PROGRAMS
  tests/count_cores.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/tests)
catkin_install_python(PROGRAMS
  core/rosbuild/bin/check_same_directories.py
  core/rosbuild/bin/download_checkmd5.py
  DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/ros/core/rosbuild/bin)
