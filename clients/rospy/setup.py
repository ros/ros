#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rospy import __version__

setup(name='rospy',
      version= __version__,
      packages=['rospy'],
      package_dir = {'':'src'},
      install_requires=['rosgraph', 'roslib'], 
      scripts = ['scripts/rospy'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rospy",
      download_url = "http://pr.willowgarage.com/downloads/rospy/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rospy Python client library for ROS", 
      long_description = """\
rospy is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters. The design of rospy favors implementation speed (i.e. developer time) over runtime performance so that algorithms can be quickly prototyped and tested within ROS. It is also ideal for non-critical-path code, such as configuration and initialization code. Many of the ROS tools are written in rospy to take advantage of the type introspection capabilities.
""",
      license = "BSD"
      )
