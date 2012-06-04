#!/usr/bin/env python

from setuptools import setup
import sys
from xml.etree.ElementTree import ElementTree

try:
    root = ElementTree(None, 'stack.xml')
    version = root.findtext('version')
except Exception, e:
    print >> sys.stderr, 'Could not extract version from your stack.xml:\n%s' % e
    sys.exit(-1)

sys.path.insert(0, 'src')

setup(name = 'roslib',
      version = version,
      packages = ['ros', 'roslib', 'rosunit', 'rosmake', 'rosclean', 'roscreate', 'rosboost_cfg'],
      package_dir = {'ros':'core/roslib/src/ros',
                     'roslib':'core/roslib/src/roslib',
                     'rosunit':'tools/rosunit/src/rosunit',
                     'rosmake':'tools/rosmake/src/rosmake',
                     'rosclean':'tools/rosclean/src/rosclean',
                     'roscreate':'tools/roscreate/src/roscreate',
                     'rosboost_cfg':'tools/rosboost_cfg/src/rosboost_cfg',
                     },
      package_data = {
          'roscreate': ['*.tmpl'],
          },
      install_requires=['rospkg'],
      scripts = ['tools/rosclean/scripts/rosclean',
                 'tools/rosmake/scripts/rosmake',
                 'tools/rosunit/scripts/rosunit',
                 'tools/rosboost_cfg/scripts/rosboost-cfg',
                 'tools/roscreate/scripts/roscreate-pkg',
                 ],
      author = "Maintained by Ken Conley",
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
