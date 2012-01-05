cmake_minimum_required(VERSION 2.8)

project(test_rosservice)
find_package(catkin)

add_service_files(DIRECTORY srv
  FILES
  HeaderEcho.srv
  )
generate_services(DEPENDENCIES std_msgs)

add_nosetests(test)
message("TODO: enable rosservice rostests")
#add_rostest(test/rosservice.test)

