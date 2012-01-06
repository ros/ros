#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

import os
import sys 
import unittest
import cStringIO
import time
        
from subprocess import Popen, PIPE, check_call, call

import rospkg

def get_test_path():
    return os.path.abspath(os.path.dirname(__file__))
def get_roswtf_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

class TestRoswtfOffline(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the rosmsg command works
    def test_cmd_help(self):
        cmd = 'roswtf'
        output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0]
        self.assert_('Options' in output)
            
    def test_offline(self):
        cmd = 'roswtf'

        # point at a different 'master'
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = 'http://localhost:11312'

        rospack = rospkg.RosPack()
        rosstack = rospkg.RosStack()
        env['ROS_PACKAGE_PATH'] = rosstack.get_path('ros_comm') + os.pathsep + rospack.get_path('std_msgs')

        cwd  = get_roswtf_path()
        kwds = { 'env': env, 'stdout': PIPE, 'stderr': PIPE, 'cwd': cwd}

        # run roswtf nakedly
        output = Popen([cmd], **kwds).communicate()
        # - due both a positive and negative test
        self.assert_('No errors or warnings' in output[0], "OUTPUT[%s]"%str(output))
        self.assert_('ERROR' not in output[0], "OUTPUT[%s]"%str(output))

        # run roswtf on a simple launch file offline
        p = os.path.join(get_test_path(), 'min.launch')
        output = Popen([cmd, p], **kwds).communicate()[0]
        self.assert_('No errors or warnings' in output, "OUTPUT[%s]"%output)
        self.assert_('ERROR' not in output, "OUTPUT[%s]"%output)        
