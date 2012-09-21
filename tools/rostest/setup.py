#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['rostest']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/rostest']
d['install_requires'] = ['rospkg', 'genmsg', 'genpy', 'roslib']

setup(**d)
