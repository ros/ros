#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

from rosmake import __version__

setup(name='rosmake',
      version= __version__,
      packages=['rosmake'],
      package_dir = {'':'src'},
      install_requires=['rospkg'], 
      scripts = ['scripts/rosmake'],
      author = "Tully Foote", 
      author_email = "tfoote@willowgarage.com",
      url = "http://www.ros.org/wiki/rosmake",
      download_url = "http://pr.willowgarage.com/downloads/rosmake/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rosmake multi-process launchers", 
      long_description = """\
rosmake is a tool for easily launching multiple ROS nodes locally and remotely via SSH, as well as setting parameters on the Parameter Server. It includes options to automatically respawn processes that have already died. rosmake takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch, as well as the machines that they should be run on.
""",
      license = "BSD"
      )
