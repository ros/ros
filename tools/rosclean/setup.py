#!/usr/bin/env python

from distutils.core import setup

import sys
sys.path.insert(0, 'src')

from genmsg import __version__

setup(name='rosclean',
      version= __version__,
      py_modules=['rosclean'],
      package_dir = {'':'src'},
      scripts = [],
      author = "Ken Conley",
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rosclean",
      download_url = "http://pr.willowgarage.com/downloads/rosclean/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosclean",
      long_description = """\
rosclean: cleanup filesystem resources (e.g. log files)
""",
      license = "BSD"
      )

