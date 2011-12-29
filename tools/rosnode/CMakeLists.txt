project(rosnode)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(rosnode
  VERSION 1.7.0
  PYTHONPATH src
  )

enable_python(rosnode)
