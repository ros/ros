cmake_minimum_required(VERSION 2.8)
project(rosout)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()
catkin_package_export()

add_executable(rosout rosout.cpp)
target_link_libraries(rosout ${catkin_LIBRARIES})

install(TARGETS rosout
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
