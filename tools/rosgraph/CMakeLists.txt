cmake_minimum_required(VERSION 2.8)
project(rosgraph)
find_package(catkin REQUIRED COMPONENTS rostest)
catkin_project(rosgraph)

add_rostest(test/rostest/masterapi.test)

# logging config file goes in both etc and package layout (for now).
# want to get rid of package layout copy, but need to be able to
# locate etc first.
install(FILES conf/python_logging.conf
  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}/ros)
install(FILES conf/python_logging.conf
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}/conf)
install(PROGRAMS scripts/rosgraph
  DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION})

catkin_add_nosetests(test)
