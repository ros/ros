#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: test_nodeprocess.py 11927 2010-10-27 08:48:11Z kwc $

PKG = 'test_roslaunch'
NAME = 'test_version'

import roslib; roslib.load_manifest(PKG)

import roslib.stacks

import os
import sys
import unittest
import re
    
def get_ros_comm_version():
    d = roslib.stacks.get_stack_dir('ros_comm')
    cmake_p = os.path.join(d, 'CMakeLists.txt')
    with open(cmake_p) as f:
        text = f.read()
    for l in text.split('\n'):
        if l.strip().startswith('rosbuild_make_distribution'):
            x_re = re.compile(r'[()]')
            lsplit = x_re.split(l.strip())
            if len(lsplit) < 2:
                raise Exception("couldn't find version number in CMakeLists.txt:\n\n%s"%l)
            return lsplit[1]
    raise Exception("could not locate version number in stack CMakeLists.txt")

class TestCoreVersion(unittest.TestCase):

    def test_version(self):
        import roslib.packages
        import roslaunch.config
        import roslaunch.xmlloader
        
        # make sure roscore version is consistent with rosversion
        rosversion = get_ros_comm_version()

        loader = roslaunch.xmlloader.XmlLoader()
        config = roslaunch.config.ROSLaunchConfig()
        d = roslib.packages.get_pkg_dir('roslaunch')
        filename = os.path.join(d, 'roscore.xml')
        loader.load(filename, config)
        core_version = config.params['/rosversion'].value

        self.assertEquals(rosversion, core_version)
    
if __name__ == '__main__':
    import rostest
    rostest.unitrun('test_roslaunch', NAME, TestCoreVersion, coverage_packages=[])
    
