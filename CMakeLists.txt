cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin)
find_package(Boost COMPONENTS thread)

# Must call catkin_export_python() early, to ensure that things like
# rosunit are symlinked into build/bin, and can be found by things like
# add_gtest(), which is called from within lower-level CMakeLists.txts.
catkin_export_python()

foreach(subdir
    env-hooks
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
install(DIRECTORY core/rosbuild/
  DESTINATION share/ros/core/rosbuild
  PATTERN ".svn" EXCLUDE
  PATTERN "bin" EXCLUDE
  )
install(PROGRAMS 
        core/rosbuild/bin/check_same_directories.py
        core/rosbuild/bin/download_checkmd5.py
        DESTINATION share/ros/core/rosbuild/bin)
install(DIRECTORY core/mk/
  DESTINATION share/ros/core/mk
  PATTERN ".svn" EXCLUDE
  )
install(FILES core/roslang/cmake/roslang.cmake
        DESTINATION share/roslang/cmake)

# install all manifests
message("TODO: exclude rosbuild/manifest.xml and mk/manifest.xml from install_matching_to_share(), because the parent directories are installed explicitly")
install_matching_to_share(manifest.xml)

# install python package-relative scripts
install(PROGRAMS 
        tools/rosunit/scripts/test_results_dir.py
        tools/rosunit/scripts/summarize_results.py
        tools/rosunit/scripts/rosunit
        tools/rosunit/scripts/pycoverage_to_html.py
        tools/rosunit/scripts/clean_junit_xml.py
        tools/rosunit/scripts/check_test_ran.py
        DESTINATION share/rosunit/bin)
install(DIRECTORY tools/roscreate/templates/
  DESTINATION share/roscreate/templates
  PATTERN ".svn" EXCLUDE
  )
