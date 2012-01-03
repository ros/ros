project(rosparam)
find_package(catkin)

add_subdirectory(scripts)

catkin_project(${PROJECT_NAME}
  PYTHONPATH src
  )

enable_python(${PROJECT_NAME})
