#!/usr/bin/env python

from __future__ import print_function
from setuptools import setup

import sys
sys.path.insert(0, 'src')

try:
    from rospy import __version__
except:
    sys.stderr.write("""

    DEPENDENCY LOOP: You'll see this message on the first run of cmake
    only, if std_msgs is in your workspace.  rospy's __init__.py
    imports std_msgs.msg.Header, and this code has not yet been
    generated.  How can we break this dependency loop?

    """)
    __version__ = "6.6.6"

sys.stderr.write("""

    TDS hackaround:  should there be a scripts/rospy?

""")

setup(name='rospy',
      version= __version__,
      packages=['rospy'],
      package_dir = {'':'src'},
      install_requires=['rosgraph', 'roslib'],
      #
      #  Hmm, this script doesn't exist...  Commented out by TDS...
      #
      #scripts = ['scripts/rospy'],
      author = "Ken Conley",
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rospy",
      download_url = "http://pr.willowgarage.com/downloads/rospy/",
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python",
        "License :: OSI Approved :: BSD License" ],
      description = "rospy Python client library for ROS",
      long_description = """\
rospy is a pure Python client library for ROS. The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters. The design of rospy favors implementation speed (i.e. developer time) over runtime performance so that algorithms can be quickly prototyped and tested within ROS. It is also ideal for non-critical-path code, such as configuration and initialization code. Many of the ROS tools are written in rospy to take advantage of the type introspection capabilities.
""",
      license = "BSD"
      )
