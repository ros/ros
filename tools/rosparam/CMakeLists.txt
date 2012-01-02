project(rosparam)
find_package(catkin)

add_subdirectory(scripts)

catkin_project(${PROJECT_NAME}
  VERSION 0.0.1
  PYTHONPATH src
  )

enable_python(${PROJECT_NAME})
