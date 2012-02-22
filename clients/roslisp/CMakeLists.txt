cmake_minimum_required(VERSION 2.8)
project(roslisp)
find_package(catkin REQUIRED)

catkin_project(roslisp
  CFG_EXTRAS roslisp-extras.cmake)

add_subdirectory(env-hooks)

# HACK(lorenz): how can wie already use roslisp-extras.cmake here?
configure_file(${roslisp_SOURCE_DIR}/cmake/roslisp-extras.cmake.in
  ${roslisp_BINARY_DIR}/cmake/roslisp-extras.cmake @ONLY)
include(${roslisp_BINARY_DIR}/cmake/roslisp-extras.cmake)

register_asdf_directory(asdf)

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
          DESTINATION ${roslisp_LISP_INSTALL_DIR}
          PATTERN ".svn" EXCLUDE
          PATTERN "genmsg_lisp.py" EXCLUDE)
endforeach()

install(DIRECTORY scripts
  DESTINATION ${roslisp_LISP_INSTALL_DIR}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE
  PATTERN "genmsg_lisp.py" EXCLUDE)
