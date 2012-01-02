project(rosunit)
find_package(catkin)

catkin_project(${PROJECT_NAME}
  VERSION 0.0.1
  PYTHONPATH src
  )

enable_python(${PROJECT_NAME})

install(DIRECTORY scripts 
  DESTINATION share/rosunit/bin
  )