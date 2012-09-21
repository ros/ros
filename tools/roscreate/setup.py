#!/usr/bin/env python

from distutils.core import setup
from catkin.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['roscreate']
d['package_dir'] = {'': 'src'}
d['install_requires' = ['rospkg']
d['scripts'] = ['scripts/roscreate-pkg']

setup(**d)
