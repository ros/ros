#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from roslaunch import __version__

setup(name='roslaunch',
      version= __version__,
      packages=['roslaunch'],
      package_dir = {'':'src'},
      install_requires=['rospkg', 'rosmaster', 'rosclean', 'rosgraph', 'rosgraph_msgs'], 
      scripts = ['scripts/roslaunch'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/roslaunch",
      download_url = "http://pr.willowgarage.com/downloads/roslaunch/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "roslaunch multi-process launchers", 
      long_description = """\
roslaunch is a tool for running multiple ROS processes easily across one or moremachines.
""",
      license = "BSD"
      )
