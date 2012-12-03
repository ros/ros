#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['rosclean']
d['package_dir'] = {'': 'src'}
d['scripts'] = ['scripts/rosclean']
d['requires'] = ['rospkg']

setup(**d)
