project(rosmsg)
find_package(catkin)

add_subdirectory(scripts)

install_cmake_infrastructure(rosmsg
  VERSION 1.7.0
  PYTHONPATH src
  )

enable_python(rosmsg)
