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

class TestRosnodeOffline(unittest.TestCase):

    def setUp(self):
        pass

    ## test that the rosmsg command works
    def test_cmd_help(self):
        cmd = 'rosnode'
        sub = ['ping', 'machine', 'list', 'info', 'kill']
        
        output = Popen([cmd], stdout=PIPE).communicate()[0]
        self.assert_('Commands' in output)
        output = Popen([cmd, '-h'], stdout=PIPE).communicate()[0]
        self.assert_('Commands' in output)
        for c in sub:
            # make sure command is in usage statement
            self.assert_("%s %s"%(cmd, c) in output)

        for c in sub:
            output = Popen([cmd, c, '-h'], stdout=PIPE, stderr=PIPE).communicate()
            self.assert_("Usage:" in output[0], "[%s]: %s"%(c, output))
            self.assert_("%s %s"%(cmd, c) in output[0], "%s: %s"%(c, output[0]))
            
        # test no args on commands that require args
        for c in ['ping', 'info']:
            output = Popen([cmd, c], stdout=PIPE, stderr=PIPE).communicate()
            self.assert_("Usage:" in output[0] or "Usage:" in output[1], "[%s]: %s"%(c, output))
            
    def test_offline(self):
        cmd = 'rosnode'

        # point at a different 'master'
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = 'http://localhost:11312'
        kwds = { 'env': env, 'stdout': PIPE, 'stderr': PIPE}

        msg = "ERROR: Unable to communicate with master!\n"

        output = Popen([cmd, 'list',], **kwds).communicate()
        self.assert_(msg in output[1])
        output = Popen([cmd, 'ping', 'talker'], **kwds).communicate()
        self.assertEquals(msg, output[1])
        output = Popen([cmd, 'info', 'talker'], **kwds).communicate()
        self.assert_(msg in output[1])

        output = Popen([cmd, 'kill', 'talker'], **kwds).communicate()
        self.assert_(msg in output[1])
        
