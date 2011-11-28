#!/usr/bin/env python
#
#  TDS 2011/11/28
#  PROTOTYPE FILE USED BY CATKIN
#  IF YOU DON'T KNOW WHAT THAT MEANS, IGNORE THIS FILE
#

from setuptools import setup

import sys
sys.path.insert(0, 'src')

#from genpy import __version__

setup(name='roslib',
      version= "6.6.6",
      packages=['roslib'],
      package_dir = {'':'src'},
      install_requires=[],
      scripts = [],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/roslib",
      download_url = "http://pr.willowgarage.com/downloads/roslib/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "ROS msg/srv Python generation", 
      long_description = """\
roslib!
""",
      license = "BSD"
      )
