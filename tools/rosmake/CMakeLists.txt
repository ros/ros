project(rosmake)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(rosmake
  VERSION 1.7.0
  PYTHONPATH src
  )

enable_python(rosmake)
