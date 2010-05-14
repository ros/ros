#!/bin/bash
set -o errexit
set -o verbose
if [ $# -ne 3 ] ; then
  echo "syntax: make-release MAJOR MINOR PATCH"
  exit 1
fi

MAJOR=$1
MINOR=$2
PATCH=$3

# copy to stable
svn delete https://ros.svn.sourceforge.net/svnroot/ros/tags/stable
svn copy https://ros.svn.sourceforge.net/svnroot/ros/tags/rc https://ros.svn.sourceforge.net/svnroot/ros/tags/stable

# copy to MAJOR.MINOR
svn delete https://ros.svn.sourceforge.net/svnroot/ros/branches/releases/ros-$MAJOR.$MINOR
svn copy https://ros.svn.sourceforge.net/svnroot/ros/tags/rc https://ros.svn.sourceforge.net/svnroot/ros/branches/releases/ros-$MAJOR.$MINOR

# copy to MAJOR.MINOR.PATCH
svn delete https://ros.svn.sourceforge.net/svnroot/ros/tags/releases/ros-$MAJOR.$MINOR.$PATCH
svn copy https://ros.svn.sourceforge.net/svnroot/ros/tags/rc https://ros.svn.sourceforge.net/svnroot/ros/tags/releases/ros-$MAJOR.$MINOR.$PATCH

# build tarball
svn co https://ros.svn.sourceforge.net/svnroot/ros/tags/releases/ros-$MAJOR.$MINOR.$PATCH ~/ros-$MAJOR.$MINOR.$PATCH
export ROS_ROOT=~/ros-$MAJOR.$MINOR.$PATCH
unset ROS_PACKAGE_PATH
cd ~/ros-$MAJOR.$MINOR.$PATCH/dist
./makedist.sh
echo "tarball built at ~/ros-$MAJOR.$MINOR.$PATCH/build/ros-$MAJOR.$MINOR.$PATCH.tar.bz2"
echo "have a nice day."

