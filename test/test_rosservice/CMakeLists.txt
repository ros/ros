cmake_minimum_required(VERSION 2.8)

project(test_rosservice)
find_package(catkin REQUIRED COMPONENTS genmsg rosunit rostest)

add_service_files(DIRECTORY srv
  FILES
  HeaderEcho.srv
  )
generate_messages(DEPENDENCIES std_msgs)

catkin_add_nosetests(test)

add_rostest(test/rosservice.test)
