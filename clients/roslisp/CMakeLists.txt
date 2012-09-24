cmake_minimum_required(VERSION 2.8)
project(roslisp)
find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS roslisp-extras.cmake)
add_subdirectory(env-hooks)

# HACK(lorenz): how can wie already use roslisp-extras.cmake here?
configure_file(${roslisp_SOURCE_DIR}/cmake/roslisp-extras.cmake.in
  ${roslisp_BINARY_DIR}/cmake/roslisp-extras.cmake @ONLY)
include(${roslisp_BINARY_DIR}/cmake/roslisp-extras.cmake)

# use roslisp_package to make the package available in roslisp's asdf
# tree.
roslisp_package(roslisp)

foreach(dir
    asdf
    load-manifest
    rosbuild
    roslisp-msg-protocol
    src
    s-xml
    s-xml-rpc
    utils)
  install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
      PATTERN ".svn" EXCLUDE
      PATTERN "genmsg_lisp.py" EXCLUDE)
endforeach()

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  PATTERN "genmsg_lisp.py" EXCLUDE)

# install legacy infrastructure needed by rosbuild
install(FILES rosbuild/roslisp.cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild)
install(PROGRAMS
        rosbuild/scripts/genmsg_lisp.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)
