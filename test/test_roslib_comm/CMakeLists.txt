cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_genmsg()
# unit tests
rosbuild_add_pyunit(test/test_md5sums.py)
rosbuild_add_pyunit(test/test_roslib_genpy.py)
rosbuild_add_pyunit(test/test_roslib_gentools.py)
rosbuild_add_pyunit(test/test_roslib_message.py)
rosbuild_add_pyunit(test/test_roslib_msgs.py)
rosbuild_add_pyunit(test/test_roslib_srvs.py)
 
