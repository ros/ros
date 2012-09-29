cmake_minimum_required(VERSION 2.8)
project(rostest)

find_package(catkin COMPONENTS rosunit)
find_package(Boost COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
  )
catkin_python_setup()

find_package(catkin COMPONENTS rostest)

catkin_add_gtest(test_permuter test/test_permuter.cpp)
if(TARGET test_permuter)
  target_link_libraries(test_permuter ${Boost_LIBRARIES})
endif()

add_rostest(test/hztest0.test)
add_rostest(test/hztest.test)
add_rostest(test/clean_master.test)
add_rostest(test/distro_version.test)

# required for old rosbuild macro
install(PROGRAMS scripts/roslaunch-check.py
  DESTINATION  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/scripts)
install(PROGRAMS nodes/hztest
  DESTINATION  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/nodes/hztest)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
