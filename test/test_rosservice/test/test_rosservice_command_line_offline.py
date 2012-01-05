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
        
import rostest

from subprocess import Popen, PIPE, check_call, call

class TestRosserviceOffline(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the usage command works
    def test_cmd_help(self):
        cmd = 'rosservice'
        sub = ['args', 'info', 'list', 'call', 'type', 'uri', 'find']
        
        output = Popen([cmd], stdout=PIPE).communicate()[0]
        self.assert_('Commands' in output)
        output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0]
        self.assert_('Commands' in output)
        # make sure all the commands are in the usage
        for c in sub:
            self.assert_("%s %s"%(cmd, c) in output, output)            

        for c in sub:
            output = Popen([cmd, c, '-h'], stdout=PIPE).communicate()
            self.assert_("Usage:" in output[0], output)
            # make sure usage refers to the command
            self.assert_("%s %s"%(cmd, c) in output[0], output)

    def test_offline(self):
        cmd = 'rosservice'

        # point at a different 'master'
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = 'http://localhost:11312'
        kwds = { 'env': env, 'stdout': PIPE, 'stderr': PIPE}

        msg = "ERROR: Unable to communicate with master!\n"

        output = Popen([cmd, 'list'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        output = Popen([cmd, 'type', 'add_two_ints'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        output = Popen([cmd, 'uri', 'add_two_ints'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        output = Popen([cmd, 'call', 'add_two_ints', '1', '2'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        # - wait should still fail if master is offline
        output = Popen([cmd, 'call', '--wait', 'add_two_ints', '1', '2'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        output = Popen([cmd, 'find', 'test_ros/AddTwoInts'], **kwds).communicate()
        self.assert_(output[1].endswith(msg))
        
if __name__ == '__main__':
    rostest.unitrun('test_rosservice', NAME, TestRosserviceOffline, sys.argv, coverage_packages=[])
