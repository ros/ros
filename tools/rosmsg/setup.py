#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosmsg import __version__

setup(name='rosmsg',
      version= __version__,
      packages=['rosmsg'],
      package_dir = {'':'src'},
      install_requires=['rospkg', 'genmsg', 'genpy', 'rosbag'], 
      scripts = ['scripts/rosmsg', 'scripts/rossrv'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosmsg",
      download_url = "http://pr.willowgarage.com/downloads/rosmsg/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosmsg multi-process launchers", 
      long_description = """\
rosmsg contains two command-line tools: rosmsg and rossrv. rosmsg is a command-line tool for displaying information about ROS Message types. rossrv is a command-line tool for displaying information about ROS Service types.
""",
      license = "BSD"
      )
