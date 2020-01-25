from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rosboost_cfg'],
    package_dir={'': 'src'},
    scripts=['scripts/rosboost-cfg']
)

setup(**d)
