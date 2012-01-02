cmake_minimum_required(VERSION 2.8)
project(std_srvs)

add_service_files(DIRECTORY srv
  FILES Empty.srv
  )

generate_messages(DEPENDENCIES std_msgs)

install_cmake_infrastructure(std_srvs
  VERSION 0.0.1
  )

