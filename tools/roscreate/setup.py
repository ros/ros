from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roscreate'],
    package_dir={'': 'src'},
    scripts=['scripts/roscreate-pkg'],
    requires=['roslib', 'rospkg']
)

setup(**d)
