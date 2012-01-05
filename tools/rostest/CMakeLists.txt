cmake_minimum_required(VERSION 2.8)
project(rostest)

add_gtest(test/test_permuter test/test_permuter.cpp)

message("TODO: enable rostest rostests)
#add_rostest(test/hztest0.test)
#add_rostest(test/hztest.test)
#add_rostest(test/clean_master.test)
#add_rostest(test/distro_version.test)

install(FILES manifest.xml
  DESTINATION share/rostest
  )

# required for old rosbuild macro
install(PROGRAMS scripts/roslaunch-check.py
  DESTINATION share/rostest/scripts
  )

install(PROGRAMS nodes/hztest
  DESTINATION share/rostest/nodes/hztest
  )
