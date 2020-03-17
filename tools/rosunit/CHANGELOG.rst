^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosunit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.15.1 (2020-03-17)
-------------------
* fix missing rosunit results in Python 3 (`#244 <https://github.com/ros/ros/issues/244>`_)

1.15.0 (2020-02-11)
-------------------

1.14.8 (2020-02-11)
-------------------
* fix various issues discovered by flake8 (`#241 <https://github.com/ros/ros/issues/241>`_)
* update style to pass flake8 (`#240 <https://github.com/ros/ros/issues/240>`_)
* Use setuptools instead of distutils (`#235 <https://github.com/ros/ros/issues/235>`_)
* Bump CMake version to avoid CMP0048 warning (`#234 <https://github.com/ros/ros/issues/234>`_)

1.14.7 (2019-10-03)
-------------------
* use condition attributes to specify Python 2 and 3 dependencies (`#226 <https://github.com/ros/ros/issues/226>`_)
* Python 3 support (`#212 <https://github.com/ros/ros/issues/212>`_)

1.14.6 (2019-03-18)
-------------------

1.14.5 (2019-03-04)
-------------------
* add Python executable to ROSUNIT_EXE so it runs correctly on Windows (`#200 <https://github.com/ros/ros/issues/200>`_)
* rosunit Python 3 support (`#190 <https://github.com/ros/ros/issues/190>`_)

1.14.4 (2018-05-01)
-------------------

1.14.3 (2018-01-30)
-------------------

1.14.2 (2017-10-26)
-------------------
* use python constants rather than hardcoded integers for error codes (`#153 <https://github.com/ros/ros/issues/153>`_)

1.14.1 (2017-07-27)
-------------------
* fix syntax of unicode raw string in Python 3 (`#150 <https://github.com/ros/ros/pull/150>`_)
* ensure cwd exists (`#143 <https://github.com/ros/ros/pull/143>`_)
* more searchable testcase result message (`#139 <https://github.com/ros/ros/pull/139>`_)

1.14.0 (2017-02-22)
-------------------

1.13.6 (2017-10-31)
-------------------
* use python constants rather than hardcoded integers for error codes (`#153 <https://github.com/ros/ros/issues/153>`_)
* fix syntax of unicode raw string in Python 3 (`#150 <https://github.com/ros/ros/pull/150>`_)
* ensure cwd exists (`#143 <https://github.com/ros/ros/pull/143>`_)
* more searchable testcase result message (`#139 <https://github.com/ros/ros/pull/139>`_)

1.13.5 (2017-02-14)
-------------------
* improve error message when creating test directory fails (`#134 <https://github.com/ros/ros/pull/134>`_)
* fix race condition creating folder (`#130 <https://github.com/ros/ros/pull/130>`_)

1.13.4 (2016-09-19)
-------------------
* fix test type handling (`#123 <https://github.com/ros/ros/issues/123>`_)

1.13.3 (2016-09-16)
-------------------
* allow custom class_name, testcase_name in test_xx_junit_xml (`#119 <https://github.com/ros/ros/issues/119>`_)
* fix check of test type (`#121 <https://github.com/ros/ros/issues/121>`_)

1.13.2 (2016-09-02)
-------------------

1.13.1 (2016-03-13)
-------------------
* fix a regression in XML reports introduced in 1.12.6 (`#109 <https://github.com/ros/ros/pull/109>`_)

1.13.0 (2016-03-10)
-------------------

1.12.6 (2016-03-10)
-------------------
* remove invalid characters from XML unit test results (`#89 <https://github.com/ros/ros/pull/89>`_, `#108 <https://github.com/ros/ros/pull/108>`_)
* add ability to load tests using dotnames in rosunit (`#101 <https://github.com/ros/ros/issues/101>`_)

1.12.5 (2015-10-13)
-------------------

1.12.4 (2015-10-12)
-------------------

1.12.3 (2015-09-19)
-------------------

1.12.2 (2015-04-27)
-------------------
* allow custom environment when determining test results location (`#82 <https://github.com/ros/ros/pull/82>`_)

1.12.1 (2015-04-16)
-------------------

1.12.0 (2014-12-26)
-------------------

1.11.6 (2014-12-22)
-------------------
* fix OSError handling (`#69 <https://github.com/ros/ros/pull/69>`_, regression since 1.11.1)

1.11.5 (2014-08-18)
-------------------

1.11.4 (2014-07-23)
-------------------

1.11.3 (2014-07-18)
-------------------

1.11.2 (2014-06-16)
-------------------

1.11.1 (2014-05-07)
-------------------
* use catkin_install_python() to install Python scripts (`#46 <https://github.com/ros/ros/issues/46>`_)
* python 3 compatibility

1.11.0 (2014-01-31)
-------------------

1.10.9 (2014-01-07)
-------------------
* python 3 compatibility
* fix repo urls in manifest

1.10.8 (2013-10-15)
-------------------

1.10.7 (2013-10-04)
-------------------
* fix sanitizing rosunit xml files on the lowest level possible

1.10.6 (2013-08-22)
-------------------

1.10.5 (2013-08-21)
-------------------
* make rosunit relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)

1.10.4 (2013-07-05)
-------------------

1.10.3 (2013-07-03)
-------------------
* check for CATKIN_ENABLE_TESTING to enable configure without tests

1.10.2 (2013-06-18)
-------------------

1.10.1 (2013-06-06)
-------------------
* make rosunit use print function for Python 2 and 3 compatibility (`#11 <https://github.com/ros/ros/issues/11>`_)
* remove unnecessary usage of unicode strings (`#12 <https://github.com/ros/ros/issues/12>`_)

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

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------

1.9.39 (2012-12-30)
-------------------
* first public release for Groovy
