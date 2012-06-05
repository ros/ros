cmake_minimum_required(VERSION 2.8)
project(ros)
find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
catkin_stack()

# Must call catkin_python_setup() early, to ensure that things like
# rosunit are available in build-space, and can be found by things like
# add_gtest(), which is called from within lower-level CMakeLists.txts.
catkin_python_setup()

foreach(subdir
    env-hooks
    tools/rosunit
    core/roslib
    tools/rosbash
    )
  add_subdirectory(${subdir})
endforeach()

# nosetests are declared here to make them visible to the build system
# (e.g., for Jenkins to invoke) but keep them from polluting the pristine
# Python packages.
find_package(ROS COMPONENTS rosunit)
add_nosetests(core/roslib/test)
add_nosetests(tools/rosmake/test)
add_nosetests(tools/rosunit/test)
add_nosetests(tools/rosclean/test)
add_nosetests(tools/roscreate/test)

# INSTALLATION
install(FILES stack.xml
        DESTINATION share/ros)
install(FILES config/rosconsole.config
        DESTINATION share/ros/config)

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
# we do them one-by-one instead of using install_matching_to_share()
# because we need to exclude some of them (e.g., mk and rosbuild are
# handled above, and some directories contains manifests that are only for
# testing.).
install(FILES core/roslang/manifest.xml DESTINATION share/roslang)
install(FILES core/roslib/manifest.xml DESTINATION share/roslib)
install(FILES tools/rosbash/manifest.xml DESTINATION share/rosbash)
install(FILES tools/rosmake/manifest.xml DESTINATION share/rosmake)
install(FILES tools/rosunit/manifest.xml DESTINATION share/rosunit)
install(FILES tools/rosclean/manifest.xml DESTINATION share/rosclean)
install(FILES tools/roscreate/manifest.xml DESTINATION share/roscreate)
install(FILES tools/rosboost_cfg/manifest.xml DESTINATION share/rosboost_cfg)

# install python package-relative scripts
install(PROGRAMS
        tools/rosunit/scripts/test_results_dir.py
        tools/rosunit/scripts/summarize_results.py
        tools/rosunit/scripts/rosunit
        tools/rosunit/scripts/pycoverage_to_html.py
        tools/rosunit/scripts/clean_junit_xml.py
        tools/rosunit/scripts/check_test_ran.py
        DESTINATION share/rosunit/bin)

install(FILES stack.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY tools/roscreate/templates/
  DESTINATION share/roscreate/templates
  PATTERN ".svn" EXCLUDE
  )
