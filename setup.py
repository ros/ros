#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

setup(name='roslib',
      version= '1.7.1',
      packages=['ros', 'roslib', 'rosunit', 'rosmake', 'rosclean', 'roscreate'],
      package_dir = {'ros':'core/roslib/src/ros',
                     'roslib':'core/roslib/src/roslib',
                     'rosunit':'tools/rosunit/src/rosunit',
                     'rosmake':'tools/rosmake/src/rosmake',
                     'rosclean':'tools/rosclean/src/rosclean',
                     'roscreate':'tools/roscreate/src/roscreate',
                     },
      install_requires=['rospkg'],
      scripts = ['tools/rosclean/scripts/rosclean',
                 'tools/rosmake/scripts/rosmake',
                 'tools/rosunit/scripts/rosunit',
                 ],
      author = "Ken Conley",
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/roslib",
      download_url = "http://pr.willowgarage.com/downloads/roslib/",
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python",
        "License :: OSI Approved :: BSD License" ],
      description = "roslib",
      long_description = """\
Internal Python libraries for low-level 'ros' stack.  This also installs the rosmake and rosclean tools.
This does *not* include the ROS communication libraries.
""",
      license = "BSD"
      )
