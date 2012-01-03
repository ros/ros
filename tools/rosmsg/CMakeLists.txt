project(rosmsg)
find_package(catkin)

add_subdirectory(scripts)

catkin_project(rosmsg
  PYTHONPATH src
  )

enable_python(rosmsg)
