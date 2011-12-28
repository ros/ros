project(rosmaster)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(rosmaster
  VERSION 1.8.0
  PYTHONPATH src
  )

enable_python(rosmaster)
