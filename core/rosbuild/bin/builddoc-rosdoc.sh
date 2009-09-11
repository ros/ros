#!/bin/bash

#############################################################
# This file is maintained in SVN at:
#   https://ros.svn.sf.net/svnroot/ros/trunk/core/rosbuild
# If you make changes here, please check them back in there.
#############################################################

#############################################################
# This script automatically build documentation, using rosdoc (which
# uses doxygen).  It checks out and runs over the repositories listed below,
# putting the results at:
#   http://pr.willowgarage.com/pr-docs/ros-packages/
#############################################################

/bin/date
tmpdir="/tmp/$(basename $0).$$.ros.tmp"
tmpdir1="/tmp/$(basename $0).$$.pr.tmp"
svn co https://ros.svn.sourceforge.net/svnroot/ros/trunk $tmpdir
svn co https://personalrobots.svn.sourceforge.net/svnroot/personalrobots/pkg/trunk $tmpdir1
export ROS_ROOT=$tmpdir
export PATH=$PATH:$ROS_ROOT/bin
export ROS_PACKAGE_PATH=$tmpdir1
cd $tmpdir/tools/rospack && make
cd $tmpdir/3rdparty/cmake && make
cd $tmpdir/std_msgs && make
make -C $tmpdir/core/rosdoc upload-local
rm -rf $tmpdir
rm -rf $tmpdir1
