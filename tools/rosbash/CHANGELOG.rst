^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbash
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.4 (2018-05-01)
-------------------
* rosrun: array is now properly expanded in debug-echo (`#176 <https://github.com/ros/ros/issues/176>`_)
* rosbash: replaced `...` with $(...) (`#177 <https://github.com/ros/ros/issues/177>`_)
* rosrun: replaced `...` with $(...) (`#175 <https://github.com/ros/ros/issues/175>`_)
* rosfish: fix syntax error (`#171 <https://github.com/ros/ros/issues/171>`_)
* fix zsh tab completion for symlinks (`#169 <https://github.com/ros/ros/issues/169>`_)

1.14.3 (2018-01-30)
-------------------
* do not remove paths containing ./ or ../ from completion (`#162 <https://github.com/ros/ros/issues/162>`_)

1.14.2 (2017-10-26)
-------------------

1.14.1 (2017-07-27)
-------------------
* add options in completion for roslaunch to roszsh (`#147 <https://github.com/ros/ros/issues/147>`_)
* allow arguments in EDITOR env in zsh rosed (`#144 <https://github.com/ros/ros/pull/144>`_)

1.14.0 (2017-02-22)
-------------------

1.13.6 (2017-10-31)
-------------------
* add options in completion for roslaunch to roszsh (`#147 <https://github.com/ros/ros/issues/147>`_)
* allow arguments in EDITOR env in zsh rosed (`#144 <https://github.com/ros/ros/pull/144>`_)

1.13.5 (2017-02-14)
-------------------
* add completion for "rosmsg info" (`#138 <https://github.com/ros/ros/pull/138>`_)
* add "rostopic pub" completion for message type (`#132 <https://github.com/ros/ros/pull/132>`_)
* fix "rostopic pub" completion when options are provided (`#131 <https://github.com/ros/ros/pull/131>`_)

1.13.4 (2016-09-19)
-------------------

1.13.3 (2016-09-16)
-------------------
* fix spelling of 'rosed' in usage (`#118 <https://github.com/ros/ros/pull/118>`_)

1.13.2 (2016-09-02)
-------------------
* add missing verbs to rosservice completion (`#117 <https://github.com/ros/ros/pull/117>`_)

1.13.1 (2016-03-13)
-------------------

1.13.0 (2016-03-10)
-------------------

1.12.6 (2016-03-10)
-------------------
* add roscat to display file contents (`#99 <https://github.com/ros/ros/pull/99>`_)
* roszsh: Ignore hidden files and directory in completion (`#100 <https://github.com/ros/ros/pull/100>`_)

1.12.5 (2015-10-13)
-------------------
* rosrun: allow spaces in command names and search paths (`#94 <https://github.com/ros/ros/pull/94>`_)

1.12.4 (2015-10-12)
-------------------
* fix zsh rosservice completion (`#92 <https://github.com/ros/ros/pull/92>`_)

1.12.3 (2015-09-19)
-------------------
* fix roslaunch completion if path contains white spaces (`ros/ros_comm#658 <https://github.com/ros/ros_comm/issues/658>`_)
* add rosconsole tab completion for bash (`#86 <https://github.com/ros/ros/pull/86>`_)
* use --first-only option when calling catkin_find (`#83 <https://github.com/ros/ros/issues/83>`_)

1.12.2 (2015-04-27)
-------------------

1.12.1 (2015-04-16)
-------------------
* add support for fish shell (`#77 <https://github.com/ros/ros/pull/77>`_)
* enable roslaunch args completion in rosbash

1.12.0 (2014-12-26)
-------------------

1.11.6 (2014-12-22)
-------------------
* match behaviour of 'roscd' in zsh with bash (`#73 <https://github.com/ros/ros/pull/73>`_)
* improve rosbag zsh tab completion for bag files (`#70 <https://github.com/ros/ros/issues/70>`_)

1.11.5 (2014-08-18)
-------------------
* fix zsh autocompletion for published topics, msg-type and YAML (`#64 <https://github.com/ros/ros/issues/64>`_)

1.11.4 (2014-07-23)
-------------------

1.11.3 (2014-07-18)
-------------------

1.11.2 (2014-06-16)
-------------------

1.11.1 (2014-05-07)
-------------------
* add rosrun --prefix, update completion (`#52 <https://github.com/ros/ros/issues/52>`_)

1.11.0 (2014-01-31)
-------------------

1.10.9 (2014-01-07)
-------------------

1.10.8 (2013-10-15)
-------------------
* fix check for permissions of executables (regression from `#37 <https://github.com/ros/ros/issues/37>`_ in 1.10.7)

1.10.7 (2013-10-04)
-------------------
* use platform dependent argument for 'find -perm' (`#33 <https://github.com/ros/ros/issues/33>`_)
* compatibility of env hooks with old workspace setup files (`#36 <https://github.com/ros/ros/issues/36>`_)
* make rosawesome more awesome
* fix return code for rospd for invalid package names (`#30 <https://github.com/ros/ros/issues/30>`_)

1.10.6 (2013-08-22)
-------------------

1.10.5 (2013-08-21)
-------------------
* make rosunit relocatable (`ros/catkin#490 <https://github.com/ros/catkin/issues/490>`_)
* fix home expansion in completion on OS X (`#27 <https://github.com/ros/ros/issues/27>`_)

1.10.4 (2013-07-05)
-------------------

1.10.3 (2013-07-03)
-------------------

1.10.2 (2013-06-18)
-------------------

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
* modified 'roscd' to switch to latest sourced catkin space when invoked without arguments (`ros/ros_comm#123 <https://github.com/ros/ros_comm/issues/123>`_)

1.9.42 (2013-01-25)
-------------------

1.9.41 (2013-01-24)
-------------------

1.9.40 (2013-01-13)
-------------------
* add 'rosnode cleanup' to autocompletion

1.9.39 (2012-12-30)
-------------------
* first public release for Groovy
