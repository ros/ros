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
roslaunch is a tool for easily launching multiple ROS nodes locally and remotely via SSH, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died. roslaunch takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.
""",
      license = "BSD"
      )
