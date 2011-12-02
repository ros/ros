#!/usr/bin/make -f
# -*- makefile -*-

#
# This file is used by catkin to make source debs.
#

# Uncomment this to turn on verbose mode.
@{
from sys import version_info as v
pyversion="%u.%u" % (v.major, v.minor)
}
export DH_VERBOSE=1
export DH_OPTIONS=-v
# this is the --install-layout=deb variety
export PYTHONPATH=@(CMAKE_INSTALL_PREFIX)/lib/python@(pyversion)/dist-packages

%:
	dh  $@@

#  you can't clean this while making packages.  it will complain that
#  clean is already made, 'make clean' doesn't normally fail if things
#  are already clean (it just becomes a noop) but 'make clean' in this
#  case wastes 'rospack', which makes it impossible to clean anything
#  else.  So we just don't bother making clean here, which means we'll 
#  get extra cruft in our packages.  Oh well.
override_dh_auto_clean:
	/bin/true

override_dh_auto_configure:
	dh_auto_configure -Scmake -- \
		-DCMAKE_INSTALL_PREFIX="@(CMAKE_INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(CMAKE_INSTALL_PREFIX)" \
		-DCATKIN_PACKAGE_PREFIX="@(CATKIN_PACKAGE_PREFIX)" \
		-DCATKIN=YES
	dh_auto_configure -Spython_distutils

override_dh_auto_install:
	dh_auto_install -Scmake
	dh_auto_install -Spython_distutils -- \
		--prefix="@(CMAKE_INSTALL_PREFIX)" --install-layout=deb

override_dh_auto_build:
	dh_auto_build -Scmake
	dh_auto_build -Spython_distutils


