#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosnode import __version__

setup(name='rosnode',
      version= __version__,
      packages=['rosnode'],
      package_dir = {'':'src'},
      install_requires=['rosgraph'], 
      scripts = ['scripts/rosnode'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosnode",
      download_url = "http://pr.willowgarage.com/downloads/rosnode/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosnode command-line tool", 
      long_description = """\
rosnode is a command-line tool for displaying debug information about ROS Nodes, including publications, subscriptions and connections. It also contains an experimental library for retrieving node information. This library is intended for internal use only.
""",
      license = "BSD"
      )
