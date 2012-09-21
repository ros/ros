#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['roslaunch']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/roslaunch',
                'scripts/roscore',
                'scripts/roslaunch-logs',
                'scripts/roslaunch-deps']
d['install_requires'] = ['rospkg', 'genmsg', 'genpy', 'roslib']

setup(**d)
