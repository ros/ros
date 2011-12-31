#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosbag import __version__

setup(name='rosbag',
      version= __version__,
      packages=['rosbag'],
      package_dir = {'':'src'},
      install_requires=['rosgraph', 'rospy'], 
      scripts = ['scripts/rosbag'],
      author = "Tim Field (tfield@willowgarage.com), Jeremy Leibs (leibs@willowgarage.com), James Bowman (jamesb@willowgarage.com)", 
      author_email = "tfield@willowgarage.com",
      url = "http://www.ros.org/wiki/rosbag",
      download_url = "http://pr.willowgarage.com/downloads/rosbag/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosbag command-line tool", 
      long_description = """\
This is a set of tools for recording from and playing back to ROS topics. It is intended to be high performance and avoids deserialization and reserialization of the messages.
""",
      license = "BSD"
      )
