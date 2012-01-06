cmake_minimum_required(VERSION 2.8)
project(std_srvs)

add_service_files(DIRECTORY srv
  FILES Empty.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_project(std_srvs)

install(DIRECTORY srv
  DESTINATION share/std_srvs
  )


