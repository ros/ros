^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roslib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.2 (2020-04-07)
-------------------

1.15.1 (2020-03-17)
-------------------

1.15.0 (2020-02-11)
-------------------

1.14.8 (2020-02-11)
-------------------
* fix various issues discovered by flake8 (`#241 <https://github.com/ros/ros/issues/241>`_)
* update style to pass flake8 (`#240 <https://github.com/ros/ros/issues/240>`_)
* restrict boost dependencies to components used (`#236 <https://github.com/ros/ros/issues/236>`_)
* Use setuptools instead of distutils (`#235 <https://github.com/ros/ros/issues/235>`_)
* Bump CMake version to avoid CMP0048 warning (`#234 <https://github.com/ros/ros/issues/234>`_)

1.14.7 (2019-10-03)
-------------------
* use condition attributes to specify Python 2 and 3 dependencies (`#226 <https://github.com/ros/ros/issues/226>`_)
* symlink search for roslib (`#225 <https://github.com/ros/ros/issues/225>`_)

1.14.6 (2019-03-18)
-------------------

1.14.5 (2019-03-04)
-------------------
* disable test with invalid dependency (`#202 <https://github.com/ros/ros/issues/202>`_)
* special handle Python script without extension on Windows (`#197 <https://github.com/ros/ros/issues/197>`_)
* use _MSC_VER for ROS_FORCE_INLINE (`#194 <https://github.com/ros/ros/issues/194>`_)
* chmod -x on Python modules (`#183 <https://github.com/ros/ros/issues/183>`_)
* fix build issue on Windows (`#186 <https://github.com/ros/ros/issues/186>`_)
* add missing dependencies (`#181 <https://github.com/ros/ros/issues/181>`_)

1.14.4 (2018-05-01)
-------------------

1.14.3 (2018-01-30)
-------------------
* replace env hooks with a dependency on ros_environment (`#166 <https://github.com/ros/ros/issues/166>`_)

1.14.2 (2017-10-26)
-------------------

1.14.1 (2017-07-27)
-------------------

1.14.0 (2017-02-22)
-------------------
* update ROS_DISTRO to lunar

1.13.6 (2017-10-31)
-------------------

1.13.5 (2017-02-14)
-------------------
* fix missing export depends (`#128 <https://github.com/ros/ros/issues/128>`_)

1.13.4 (2016-09-19)
-------------------

1.13.3 (2016-09-16)
-------------------
* allow expected ROS_DISTRO value to be overridden at compile time (`#122 <https://github.com/ros/ros/pull/122>`_)

1.13.2 (2016-09-02)
-------------------
* avoid putting the rosbuild stacks dir on RPP if it doesn't exist (`#111 <https://github.com/ros/ros/pull/111>`_)

1.13.1 (2016-03-13)
-------------------

1.13.0 (2016-03-10)
-------------------
* update ROS_DISTRO to kinetic

1.12.6 (2016-03-10)
-------------------
* expose an API (ros::package::getPlugins) which can map multiple export values to one package name (`#103 <https://github.com/ros/ros/issues/103>`_)
* deprecate API returning incomplete information (`#103 <https://github.com/ros/ros/issues/103>`_)
* allow caching of rospack results (`#97 <https://github.com/ros/ros/issues/97>`_)

1.12.5 (2015-10-13)
-------------------

1.12.4 (2015-10-12)
-------------------
* improve performance by caching the package mapping in the rospack instance (`#95 <https://github.com/ros/ros/pull/95>`_)

1.12.3 (2015-09-19)
-------------------

1.12.2 (2015-04-27)
-------------------

1.12.1 (2015-04-16)
-------------------
* remove usage of CATKIN_TEST_RESULTS_DIR environment variable (`#80 <https://github.com/ros/ros/pull/80>`_)

1.12.0 (2014-12-26)
-------------------
* update ROS_DISTRO to jade

1.11.6 (2014-12-22)
-------------------
* consider std_msgs/Header to be a valid header in rosbuild-based messages (`#67 <https://github.com/ros/ros/pull/67>`_)

1.11.5 (2014-08-18)
-------------------

1.11.4 (2014-07-23)
-------------------

1.11.3 (2014-07-18)
-------------------
* remove linking against "rt" library on Android (`#57 <https://github.com/ros/ros/issues/57>`_)
* disable delayed expansion in Windows environment hook (`#60 <https://github.com/ros/ros/issues/60>`_)

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
