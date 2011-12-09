cmake_minimum_required(VERSION 2.8)
project(rosbash)
install(FILES rosbash rostcsh roszsh
        DESTINATION share/ros/rosbash)
# TODO: manifests should go in automatically
install(FILES manifest.xml
        DESTINATION share/ros/rosbash)
