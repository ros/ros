cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_library(XmlRpc src/XmlRpcClient.cpp
                     src/XmlRpcDispatch.cpp
                     src/XmlRpcServer.cpp
                     src/XmlRpcServerConnection.cpp
                     src/XmlRpcServerMethod.cpp
                     src/XmlRpcSocket.cpp
                     src/XmlRpcSource.cpp
                     src/XmlRpcUtil.cpp
                     src/XmlRpcValue.cpp)

