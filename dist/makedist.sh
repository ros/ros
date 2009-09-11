#!/bin/bash

# Dirty hacks for prototyping a distro system

# We need rospack built to find and clean everything
cd $ROS_ROOT/tools/rospack && make

# We're only dealing with packages in ROS_ROOT
unset ROS_PACKAGE_PATH

# Clean everything; this seems to be the only way to reliably avoid
# having cpack include build artifacts in the source distro.
for p in `rospack list-names`; do
  echo -n "Cleaning $p..."
  if [ "x$p" != "xrospack" ]; then
    cd `rospack find $p` && make clean
  fi
  echo "...done"
done
  
set -x

# Make sure 3rdparty is totally wiped, removing tarballs, etc.
cd $ROS_ROOT/3rdparty && make clean

# Promote the distro cmake file to the top and call it
cp $ROS_ROOT/dist/CMakeLists.txt $ROS_ROOT
rm -rf $ROS_ROOT/build
mkdir -p $ROS_ROOT/build
touch $ROS_ROOT/build/rospack_nosubdirs
cd $ROS_ROOT/build && cmake ..

# We're done calling rospack now
cd $ROS_ROOT/tools/rospack && make clean

# Make sure the top-level build is clean and create the source tarball
cd $ROS_ROOT/build && make clean
cd $ROS_ROOT/build && make package_source

# Remove the distro cmake file, to prevent confusion
rm $ROS_ROOT/CMakeLists.txt

# For the user's convenience, rebuild rospack
cd $ROS_ROOT/tools/rospack && make
