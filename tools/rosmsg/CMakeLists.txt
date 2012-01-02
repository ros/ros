project(rosmsg)
find_package(catkin)

add_subdirectory(scripts)

catkin_project(rosmsg
  VERSION 1.7.0
  PYTHONPATH src
  )

enable_python(rosmsg)
