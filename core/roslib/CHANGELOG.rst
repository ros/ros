^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.2 (2014-06-16)
-------------------
* use PYTHON_EXECUTABLE in environment hooks (`#55 <https://github.com/ros/ros/issues/55>`_)

1.11.1 (2014-05-07)
-------------------
* add optional argument force_recrawl to getPlugins() function
* use catkin_install_python() to install Python scripts (`#46 <https://github.com/ros/ros/issues/46>`_)
* python 3 compatibility

1.11.0 (2014-01-31)
-------------------

1.10.9 (2014-01-07)
-------------------

1.10.8 (2013-10-15)
-------------------

1.10.7 (2013-10-04)
-------------------
* compatibility of env hooks with old workspace setup files (`#36 <https://github.com/ros/ros/issues/36>`_)
* allow python files to pass executable filter in Windows

1.10.6 (2013-08-22)
-------------------
* fix regression of `#29 <https://github.com/ros/ros/issues/29>`_ introduced in 1.10.5

1.10.5 (2013-08-21)
-------------------
* make roslib relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)
* improve performance of dry message generation

1.10.4 (2013-07-05)
-------------------

1.10.3 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.10.2 (2013-06-18)
-------------------

1.10.1 (2013-06-06)
-------------------

1.10.0 (2013-03-22 09:23)
-------------------------
* update ROS distro name to hydro (`#10 <https://github.com/ros/ros/issues/10>`_)

1.9 (Groovy)
============

1.9.44 (2013-03-13)
-------------------

1.9.43 (2013-03-08)
-------------------
* improve speed of message generation in dry packages (`ros/ros_comm#183 <https://github.com/ros/ros_comm/issues/183>`_)
* fix handling spaces in folder names (`ros/catkin#375 <https://github.com/ros/catkin/issues/375>`_)
* make Python scripts executable from launch files on Windows (`ros/ros_comm#54 <https://github.com/ros/ros_comm/issues/54>`_)

1.9.42 (2013-01-25)
-------------------
* fix location of (obsolete) environment variable ROS_ETC_DIR

1.9.41 (2013-01-24)
-------------------
* modified ROS_ROOT in devel space and moved all rosbuild files to a location which fits how the files are relatively looked up

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-30)
-------------------
* first public release for Groovy
