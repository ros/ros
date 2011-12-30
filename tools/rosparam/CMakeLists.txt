project(rosparam)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(rosparam
  VERSION 1.7.0
  PYTHONPATH src
  )

enable_python(rosparam)
