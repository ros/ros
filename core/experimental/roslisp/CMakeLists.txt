cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(roslisp)
file(MAKE_DIRECTORY lib)
set(imagename "${PROJECT_SOURCE_DIR}/lib/roslisp.image")
# TODO: make this generation of dependencies smarter; roslisp.image really 
# shouldn't depend on *.lisp, as found recursively in this directory
file(GLOB_RECURSE lispfiles ${PROJECT_SOURCE_DIR} "*.lisp")
add_custom_command(OUTPUT ${imagename}
                   COMMAND ${PROJECT_SOURCE_DIR}/scripts/make_roslisp_image ${imagename}
                   DEPENDS ${lispfiles})
add_custom_target(image ALL
                  DEPENDS ${imagename})

