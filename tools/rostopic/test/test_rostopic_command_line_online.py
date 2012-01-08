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
import rostest

import std_msgs.msg

from subprocess import Popen, PIPE, check_call, call

def run_for(cmd, secs):
    popen = Popen(cmd, stdout=PIPE, stderr=PIPE, close_fds=True)
    timeout_t = time.time() + secs
    while time.time() < timeout_t:
        time.sleep(0.1)
    os.kill(popen.pid, signal.SIGKILL)
    
class TestRostopicOnline(unittest.TestCase):

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def callback(self, msg, val):
        self.vals.add(val)
        self.msgs[val] = msg
        
    def test_rostopic(self):
        topics = ['/chatter', '/foo/chatter', '/bar/chatter']
        
        # wait for network to initialize
        rospy.init_node('test')
        for i, t in enumerate(topics):
            rospy.Subscriber(t, std_msgs.msg.String, self.callback, i)
        all = set(range(0, len(topics)))

        timeout_t = time.time() + 10.
        while time.time() < timeout_t and self.vals != all:
            time.sleep(0.1)

        # network is initialized
        cmd = 'rostopic'
        names = ['/chatter', 'foo/chatter']

        # list
        # - we aren't matching against the core services as those can make the test suites brittle
        output = Popen([cmd, 'list'], stdout=PIPE).communicate()[0]
        l = set(output.split())
        for t in topics:
            self.assert_(t in l)

        for name in names:
            # type
            output = Popen([cmd, 'type', name], stdout=PIPE).communicate()[0]
            self.assertEquals('std_msgs/String', output.strip())

            # find
            output = Popen([cmd, 'find', 'std_msgs/String'], stdout=PIPE).communicate()[0]
            values = [n.strip() for n in output.split('\n') if n.strip()]
            self.assertEquals(set(values), set(topics))

            #echo
            # test with -c option to get command to terminate
            count = 3
            output = Popen([cmd, 'echo', name, '-n', str(count)], stdout=PIPE).communicate()[0]
            values = [n.strip() for n in output.split('\n') if n.strip()]
            values = [n for n in values if n != '---']
            self.assertEquals(count, len(values), "wrong number of echos in output:\n"+str(values))
            for n in values:
                self.assert_('data: hello world ' in n, n)

            if 0:
                #bw
                stdout, stderr = run_for([cmd, 'bw', name], 3.)
                self.assert_('average:' in stdout, "OUTPUT: %s\n%s"%(stdout,stderr))

                # hz
                stdout, stderr = run_for([cmd, 'hz', name], 2.)
                self.assert_('average rate:' in stdout)
            
        # pub
        #  - pub wait until ctrl-C, so we have to wait then kill it
        if 1:
            s = 'hello'
            t = '/pub/chatter'
            key = len(topics)
            rospy.Subscriber(t, std_msgs.msg.String, self.callback, key)

            #TODO: correct popen call
            args = [cmd, 'pub', t, 'std_msgs/String', s]
            popen = Popen(args, stdout=PIPE, stderr=PIPE, close_fds=True)
        
            # - give rostopic pub 5 seconds to send us a message
            all = set(range(0, key+1))
            timeout_t = time.time() + 5.
            while time.time() < timeout_t and self.vals != all:
                time.sleep(0.1)
            # - check published value
            msg = self.msgs[key]
            self.assertEquals(s, msg.data)
            
            os.kill(popen.pid, signal.SIGKILL)

            # test with dictionary
            t = '/pub2/chatter'
            key = len(topics)+1            
            rospy.Subscriber(t, std_msgs.msg.String, self.callback, key)

            args = [cmd, 'pub', t, 'std_msgs/String', "{data: %s}"%s]
            popen = Popen(args, stdout=PIPE, stderr=PIPE, close_fds=True)

            # - give rostopic pub 5 seconds to send us a message
            all = set(range(0, key+2))
            timeout_t = time.time() + 5.
            while time.time() < timeout_t and self.vals != all:
                time.sleep(0.1)
                
            # - check published value
            try:
                msg = self.msgs[key]
            except KeyError:
                self.fail("no message received on "+str(key))
            self.assertEquals(s, msg.data)
            
            os.kill(popen.pid, signal.SIGKILL)
            
PKG = 'test_rostopic'
NAME = 'test_rostopic_command_line_online'
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestRostopicOnline, sys.argv)
