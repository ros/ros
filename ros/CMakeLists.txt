cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin REQUIRED)
catkin_package()

catkin_add_env_hooks(10.ros SHELLS bat sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
