^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbuild
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.5 (2014-08-18)
-------------------

1.11.4 (2014-07-23)
-------------------
* fix dry rosboost_cfg to support finding boost in multiarch lib folder (`#62 <https://github.com/ros/ros/issues/62>`_)

1.11.3 (2014-07-18)
-------------------
* suppress warning for rosbuild target "test" with CMake 3.0 (`#61 <https://github.com/ros/ros/issues/61>`_)

1.11.2 (2014-06-16)
-------------------

1.11.1 (2014-05-07)
-------------------
* fix CMake warning with 2.8.12 and newer (`#44 <https://github.com/ros/ros/issues/44>`_)
* use catkin_install_python() to install Python scripts (`#46 <https://github.com/ros/ros/issues/46>`_)
* python 3 compatibility

1.11.0 (2014-01-31)
-------------------
* ensure escaping of preprocessor definition (`#43 <https://github.com/ros/ros/issues/43>`_)

1.10.9 (2014-01-07)
-------------------

1.10.8 (2013-10-15)
-------------------

1.10.7 (2013-10-04)
-------------------
* compatibility of env hooks with old workspace setup files (`#36 <https://github.com/ros/ros/issues/36>`_)

1.10.6 (2013-08-22)
-------------------

1.10.5 (2013-08-21)
-------------------
* make rosbuild relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)

1.10.4 (2013-07-05)
-------------------

1.10.3 (2013-07-03)
-------------------

1.10.2 (2013-06-18)
-------------------
* update rosbuild to use moved roslaunch-check script (`ros/ros_comm#241 <https://github.com/ros/ros_comm/issues/241>`_)

1.10.1 (2013-06-06)
-------------------

1.10.0 (2013-03-22 09:23)
-------------------------

1.9 (Groovy)
============

1.9.44 (2013-03-13)
-------------------

1.9.43 (2013-03-08)
-------------------
* fix handling spaces in folder names (`ros/catkin#375 <https://github.com/ros/catkin/issues/375>`_)

1.9.42 (2013-01-25)
-------------------
* fix install location of relocated rosbuild stuff

1.9.41 (2013-01-24)
-------------------
* modified ROS_ROOT in devel space and moved all rosbuild files to a location which fits how the files are relatively looked up
* modified install location of download_checkmd5 script to work in devel space and be consistent with other files
* fix wrong comments about location of rosconfig.cmake

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-30)
-------------------
* first public release for Groovy
