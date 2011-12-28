project(roslaunch)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(roslaunch
  VERSION 1.8.0
  PYTHONPATH src
  )

enable_python(roslaunch)
