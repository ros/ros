cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin)
find_package(Boost COMPONENTS thread)

catkin_add_env_hooks(10.ros.buildspace.sh 10.ros.sh)

# Must call catkin_export_python() early, to ensure that things like
# rosunit are symlinked into build/bin, and can be found by things like
# add_gtest(), which is called from within lower-level CMakeLists.txts.
catkin_export_python()

foreach(subdir
    core/roslib
    tools/rosbash
    )
  add_subdirectory(${subdir})
endforeach()

# nosetests are declared here to make them visible to the build system
# (e.g., for Jenkins to invoke) but keep them from polluting the pristine
# Python packages.
add_nosetests(core/roslib/test)
add_nosetests(tools/rosmake/test)
add_nosetests(tools/rosunit/test)
add_nosetests(tools/rosclean/test)
add_nosetests(tools/roscreate/test)

# INSTALLATION

# setup our backwards-compatibility ROS_ROOT
install(DIRECTORY core/rosbuild
  DESTINATION share/ros/core/rosbuild
  )
install(DIRECTORY core/mk
  DESTINATION share/ros/core/rosbuild
  )

# install all manifests
install_matching_to_share(manifest.xml)

# install python package-relative scripts
install(DIRECTORY tools/rosunit/scripts
  DESTINATION share/rosunit/bin
  )
install(DIRECTORY tools/roscreate/templates
  DESTINATION share/roscreate
  )
