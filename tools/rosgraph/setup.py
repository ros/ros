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

setup(name='rosgraph',
      version= "6.6.6",
      packages=['rosgraph'],
      package_dir = {'':'src'},
      install_requires=[],
      scripts = [],
      author = "Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosgraph",
      download_url = "http://pr.willowgarage.com/downloads/rosgraph/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosgraph command-line tool",
      long_description = """\
rosgraph contains the rosgraph command-line tool, which prints information about the ROS Computation Graph. It also provides an internal library that is used by the graphical version of this tool, <tt>rxgraph</tt>.
""",
      license = "BSD"
      )

