project(rosmaster)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  PYTHONPATH src
  )

enable_python(${PROJECT_NAME})
