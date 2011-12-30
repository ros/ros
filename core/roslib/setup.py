#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from roslib import __version__

setup(name='roslib',
      version= __version__,
      packages=['roslib'],
      package_dir = {'':'src'},
      install_requires=['rospkg'], 
      scripts = ['scripts/gendeps'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/roslib",
      download_url = "http://pr.willowgarage.com/downloads/roslib/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "roslib", 
      long_description = """\
This is a deprecated library that is being phased out starting in the ROS Fuerte release.  It contains base dependencies and support libraries for ROS. roslib contains many of the common data structures and tools that are shared across ROS client library implementations.
""",
      license = "BSD"
      )
