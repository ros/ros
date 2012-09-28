#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['message_filters']
d['package_dir'] = {'': 'src'}
d['install_requires'] = ['genmsg', 'genpy', 'roslib', 'rospkg']

setup(**d)
