cmake_minimum_required(VERSION 2.8)
project(mk)
find_package(catkin REQUIRED)
catkin_package()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.awk"
  PATTERN "*.mk"
  PATTERN "*.py"
  PATTERN "*.xml")
