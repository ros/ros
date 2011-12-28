#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosmaster import __version__

setup(name='rosmaster',
      version= __version__,
      packages=['rosmaster'],
      package_dir = {'':'src'},
      install_requires=[], # TODO: roslib
      scripts = ['scripts/rosmaster'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosmaster",
      download_url = "http://pr.willowgarage.com/downloads/rosmaster/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "ROS master implementation (Python)", 
      long_description = """\
ROS Master implementation in Python.
""",
      license = "BSD"
      )
