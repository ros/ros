#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rostopic import __version__

setup(name='rostopic',
      version= __version__,
      packages=['rostopic'],
      package_dir = {'':'src'},
      install_requires=['roslib', 'rosgraph', 'rospy', 'rosmsg', 'rosbag'], 
      scripts = ['scripts/rostopic'],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rostopic",
      download_url = "http://pr.willowgarage.com/downloads/rostopic/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rostopic command-line tool", 
      long_description = """\
rostopic contains the rostopic command-line tool for displaying debug information about ROS Topics, including publishers, subscribers, publishing rate, and ROS Messages. It also contains an experimental Python library for getting information about and interacting with topics dynamically. This library is for internal-use only as the code API may change, though it does provide examples of how to implement dynamic subscription and publication behaviors in ROS.
""",
      license = "BSD"
      )
