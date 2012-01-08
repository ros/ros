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
import signal
import sys 
import time
import unittest

import rospy
import std_msgs.msg
import rostest

from subprocess import Popen, PIPE, check_call, call

def run_for(cmd, secs):
    popen = Popen(cmd, stdout=PIPE, stderr=PIPE, close_fds=True)
    timeout_t = time.time() + secs
    while time.time() < timeout_t:
        time.sleep(0.1)
    os.kill(popen.pid, signal.SIGKILL)
    
class TestRosnodeOnline(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def callback(self, msg, val):
        self.vals.add(val)
        self.msgs[val] = msg
        
    def test_rosnode(self):
        topics = ['/chatter', '/foo/chatter', '/bar/chatter']
        
        # wait for network to initialize
        rospy.init_node('test')
        nodes = ['/talker', '/foo/talker', '/bar/talker', rospy.get_caller_id()]
        
        for i, t in enumerate(topics):
            rospy.Subscriber(t, std_msgs.msg.String, self.callback, i)
        all = set(range(0, len(topics)))

        timeout_t = time.time() + 10.
        while time.time() < timeout_t and self.vals != all:
            time.sleep(0.1)
        self.assertEquals(self.vals, all, "failed to initialize graph correctly")
            

        # network is initialized
        cmd = 'rosnode'

        # list
        # - we aren't matching against the core services as those can make the test suites brittle
        output = Popen([cmd, 'list'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        for t in nodes:
            self.assert_(t in l, "%s not in %s"%(t, l))

        output = Popen([cmd, 'list', '-a'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        for t in nodes:
            for e in l:
                if t in e:
                    break
            else:
                self.fail("did not find [%s] in list [%s]"%(t, l))

        output = Popen([cmd, 'list', '-u'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        self.assert_(len(l), "list -u is empty")
        for e in l:
            self.assert_(e.startswith('http://'))

        for name in nodes:
            # type
            output = Popen([cmd, 'info', name], stdout=PIPE).communicate()[0]
            # not really validating output as much as making sure it's not broken
            self.assert_(name in output)
            self.assert_('chatter' in output)
            self.assert_('Publications' in output)
            self.assert_('Subscriptions' in output)                        

            if 0:
                #ping
                stdout, stderr = run_for([cmd, 'ping', name], 3.)

PKG = 'test_rosnode'
NAME = 'test_rosnode_command_line_online'
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRosnodeOnline, sys.argv)
