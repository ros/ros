cmake_minimum_required(VERSION 2.8)
project(std_srvs)
find_package(catkin REQUIRED COMPONENTS genmsg)
catkin_package()

add_service_files(DIRECTORY srv
  FILES Empty.srv)

generate_messages()
