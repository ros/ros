cmake_minimum_required(VERSION 3.0.2)
project(mk)
find_package(catkin REQUIRED)
catkin_package()

# do not wildcard install files since the root folder of the package will contain a debian folder for releasing
install(FILES
  buildtest.mk
  bzr_checkout.mk
  cmake.mk
  cmake_stack.mk
  download_unpack_build.mk
  eclipse.awk
  git_checkout.mk
  hg_checkout.mk
  make_pydev_project.py
  svn_checkout.mk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
