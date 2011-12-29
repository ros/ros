#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosparam import __version__

setup(name='rosparam',
      version= __version__,
      packages=['rosparam'],
      package_dir = {'':'src'},
      install_requires=['rospkg', 'rosgraph'], 
      scripts = ['scripts/rosparam'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosparam",
      download_url = "http://pr.willowgarage.com/downloads/rosparam/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosparam command-line tool", 
      long_description = """\
rosparam contains the rosparam command-line tool for getting and setting ROS Parameters on the Parameter Server using YAML-encoded files. It also contains an experimental library for using YAML with the Parameter Server. This library is intended for internal use only.
""",
      license = "BSD"
      )
