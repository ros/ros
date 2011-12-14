cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin)
install_cmake_infrastructure(${PROJECT_NAME}
  VERSION 0.0.1
  PYTHONPATH src
  )
enable_python(${PROJECT_NAME})

# Double-installation, for global access, and package-relative (e.g.,
# roslaunch) access
install(PROGRAMS nodes/rosgraph nodes/rxgraph
        DESTINATION bin)
install(PROGRAMS nodes/rosgraph nodes/rxgraph
        DESTINATION share/ros/${PROJECT_NAME}/bin)
