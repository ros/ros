#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['roslaunch']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/roscore',
                'scripts/roslaunch',
                'scripts/roslaunch-deps',
                'scripts/roslaunch-logs']
d['install_requires'] = ['genmsg', 'genpy', 'roslib', 'rospkg']

setup(**d)
