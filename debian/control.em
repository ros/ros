Source: @(CATKIN_PACKAGE_PREFIX)ros-comm
Section: misc
Priority: extra
Maintainer: Troy Straszheim <straszheim@@willowgarage.com>
Build-Depends: debhelper (>= 7), cmake, gcc, make, catkin
Homepage: <insert the upstream URL, if relevant>

Package: @(CATKIN_PACKAGE_PREFIX)ros-comm
Architecture: any
Depends: ${misc:Depends}
Description: <insert up to 60 chars description>
 <insert long description, indented with spaces>
X-ROS-Pkg-Name: ros_comm
X-ROS-Pkg-Depends: catkin, roscpp_core, std_msgs
X-ROS-System-Depends:
