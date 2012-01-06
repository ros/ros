cmake_minimum_required(VERSION 2.8)
project(rosbash)
install(FILES rosbash rostcsh roszsh
        DESTINATION share/ros/rosbash)
install(PROGRAMS scripts/rosrun
        DESTINATION bin)
