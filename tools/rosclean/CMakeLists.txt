project(rosclean)
find_package(catkin)
install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  PYTHONPATH src
  )
enable_python(${PROJECT_NAME})
