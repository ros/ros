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
import time
import unittest

import cStringIO
from subprocess import Popen, PIPE, check_call, call

from contextlib import contextmanager
@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = cStringIO.StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout

def tolist(b):
    return [x.strip() for x in b.getvalue().split('\n') if x.strip()]

class TestRosnode(unittest.TestCase):

    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        self.vals = set()
        
        # wait for network to initialize
        import rospy
        import std_msgs.msg
        rospy.init_node('test')
        topics = ['/chatter', '/foo/chatter', '/bar/chatter']        
        subs = [rospy.Subscriber(t, std_msgs.msg.String, self.callback, i) for i, t in enumerate(topics)]
        all = set(range(0, len(topics)))

        timeout_t = time.time() + 10.
        while time.time() < timeout_t and self.vals != all:
            time.sleep(0.1)
        [s.unregister() for s in subs]

    def setUp(self):
        self.vals = set()
        self.msgs = {}

    def callback(self, msg, val):
        self.vals.add(val)

    def _check(self, expected, actual):
        """
        Make sure all elements of expected are present in actual
        """
        for t in expected:
            self.assert_(t in actual)
            
    def _notcheck(self, not_expected, actual):
        """
        Make sure all elements of not_expected are not present in actual
        """
        for t in not_expected:
            self.failIf(t in actual)
        

    def test_rosnode_info(self):
        import rosnode
        cmd = 'rosnode'

        nodes = ['/talker',
                 '/foo/talker',
                 '/bar/talker',
                 '/baz/talker1',
                 '/baz/talker2',
                 '/baz/talker3',
                 '/listeners/listener',
                 '/rosout',
                 ]
        try:
            rosnode._rosnode_cmd_info([cmd, 'info'])
            self.fail("should have failed")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
            
        for n in nodes:
            with fakestdout() as b:
                rosnode._rosnode_cmd_info([cmd, 'info', n])
                s = b.getvalue()
                self.assert_("Node [%s]"%n in s)
                self.assert_("Pid: " in s, s)
        
    def test_rosnode_list(self):
        import rosnode
        cmd = 'rosnode'

        nodes = ['/talker',
                 '/foo/talker',
                 '/bar/talker',
                 '/baz/talker1',
                 '/baz/talker2',
                 '/baz/talker3',
                 '/rosout',
                 ]
        l = rosnode.get_node_names()
        for t in nodes:
            self.assert_(t in l)
            
        try:
            rosnode._rosnode_cmd_list([cmd, 'list', 'one', 'two'])
            self.fail("should have failed")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
            
        with fakestdout() as b:
            rosnode._rosnode_cmd_list([cmd, 'list'])
            self._check(nodes, tolist(b))
        with fakestdout() as b:
            rosnode._rosnode_cmd_list([cmd, 'list', '/'])
            l = tolist(b)
            self._check(nodes, l)
            num_nodes = len(l)
        # test -u uris
        with fakestdout() as b:
            rosnode._rosnode_cmd_list([cmd, 'list', '-u', '/'])
            l = tolist(b)
            self.assertEquals(num_nodes, len(l))
            self.failIf([n for n in l if not n.startswith('http://')])
        # test -a all
        with fakestdout() as b:
            rosnode._rosnode_cmd_list([cmd, 'list', '-a', '/'])
            l = tolist(b)
            uris = [x.split()[0] for x in l if x]
            names = [x.split()[1] for x in l if x]
            self._check(nodes, names) 
            self.assertEquals(num_nodes, len(uris))
            self.failIf([n for n in uris if not n.startswith('http://')])
            
            
        # test with namespace
        foon = [p for p in nodes if p.startswith('/foo/')]
        not_foon = [p for p in nodes if not p.startswith('/foo/')]
        for ns in ['foo', '/foo', '/foo/']:
            with fakestdout() as b:
                rosnode._rosnode_cmd_list([cmd, 'list', ns])
                self._check(foon, tolist(b))
                self._notcheck(not_foon, tolist(b))
        bazn = [p for p in nodes if p.startswith('/baz/')]
        not_bazn = [p for p in nodes if not p.startswith('/baz/')]
        for ns in ['baz', '/baz', '/baz/']:
            with fakestdout() as b:
                rosnode._rosnode_cmd_list([cmd, 'list', ns])
                self._check(bazn, tolist(b))
                self._notcheck(not_bazn, tolist(b))
            
        # test with no match        
        with fakestdout() as b:
            rosnode._rosnode_cmd_list([cmd, 'list', '/not/a/namespace/'])
            self.assertEquals([], tolist(b))
            

    def test_rosnode_usage(self):
        import rosnode
        cmd = 'rosnode'
        for c in ['ping', 'list', 'info', 'machine', 'cleanup', 'kill']:
            try:
                with fakestdout() as b:
                    rosnode.rosnodemain([cmd, c, '-h'])
                    self.assert_("usage" in b.getvalue())
                self.fail("should have exited on usage")
            except SystemExit, e:
                self.assertEquals(0, e.code)
            
    def test_rosnode_ping(self):
        import rosnode
        cmd = 'rosnode'
        
        self.failIf(rosnode.rosnode_ping('/fake_node', max_count=1))
        self.assert_(rosnode.rosnode_ping('/rosout', max_count=1))
        self.assert_(rosnode.rosnode_ping('/rosout', max_count=2))        

        with fakestdout() as b:
            self.assert_(rosnode.rosnode_ping('/rosout', max_count=2, verbose=True))
            s = b.getvalue()
            self.assert_('xmlrpc reply' in s, s)
            self.assert_('ping average:' in s, s)
            
        # test via command-line API
        rosnode._rosnode_cmd_ping([cmd, 'ping', '-c', '1', '/fake_node'])
        with fakestdout() as b:
            rosnode._rosnode_cmd_ping([cmd, 'ping', '-c', '1', '/rosout'])
            s = b.getvalue()
            self.assert_('xmlrpc reply' in s, s)
        with fakestdout() as b:
            rosnode._rosnode_cmd_ping([cmd, 'ping', '-c', '1', 'rosout'])
            s = b.getvalue()
            self.assert_('xmlrpc reply' in s, s)
        with fakestdout() as b:
            rosnode._rosnode_cmd_ping([cmd, 'ping', '-c', '2', 'rosout'])
            s = b.getvalue()
            self.assertEquals(2, s.count('xmlrpc reply'))
        
    def test_rosnode_ping_all(self):
        import rosnode
        cmd = 'rosnode'
        
        pinged, unpinged = rosnode.rosnode_ping_all(verbose=False)
        self.assert_('/rosout' in pinged)
        with fakestdout() as b:
            pinged, unpinged = rosnode.rosnode_ping_all(verbose=True)
            self.assert_('xmlrpc reply' in b.getvalue())
            self.assert_('/rosout' in pinged)
            
    def test_rosnode_kill(self):
        import rosnode
        cmd = 'rosnode'
        for n in ['to_kill/kill1', '/to_kill/kill2']:
            self.assert_(rosnode.rosnode_ping(n, max_count=1))
            rosnode._rosnode_cmd_kill([cmd, 'kill', n])
            self.failIf(rosnode.rosnode_ping(n, max_count=1))
        
    def test_fullusage(self):
        import rosnode
        try:
            rosnode._fullusage()
        except SystemExit: pass
        try:
            rosnode.rosnodemain(['rosnode'])
        except SystemExit: pass
        try:
            rosnode.rosnodemain(['rosnode', 'invalid'])        
        except SystemExit: pass
