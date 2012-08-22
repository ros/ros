cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin REQUIRED COMPONENTS rostest)

add_rostest(test/rostest/masterapi.test)

# logging config file goes in both etc and package layout (for now).
# want to get rid of package layout copy, but need to be able to
# locate etc first.
install(FILES conf/python_logging.conf
  DESTINATION etc/ros
  )
install(FILES conf/python_logging.conf
  DESTINATION share/rosgraph/conf
  )
install(PROGRAMS scripts/rosgraph
  DESTINATION share/rosgraph/scripts
  )
