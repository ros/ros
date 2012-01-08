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
        
import rospy
import std_msgs.msg

from subprocess import Popen, PIPE, check_call, call

from contextlib import contextmanager

@contextmanager
def fakestdout():
    realstdout = sys.stdout
    fakestdout = cStringIO.StringIO()
    sys.stdout = fakestdout
    yield fakestdout
    sys.stdout = realstdout
    
def todict(s):
    d = {}
    for l in s.split('\n'):
        key, p, val = l.partition(':')
        if p:
            d[key] = val.strip()
    return d
        
class TestRostopic(unittest.TestCase):

    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        self.vals = set()
        
        # wait for network to initialize
        rospy.init_node('test')
        topics = ['/chatter', '/foo/chatter', '/bar/chatter']        
        subs = [rospy.Subscriber(t, std_msgs.msg.String, self.callback, i) for i, t in enumerate(topics)]
        all = set(range(0, len(topics)))

        timeout_t = time.time() + 10.
        while time.time() < timeout_t and self.vals != all and not rospy.is_shutdown():
            time.sleep(0.1)
        [s.unregister() for s in subs]
        if rospy.is_shutdown():
            self.fail("shutdown")
            
    def test_offline(self):
        import rostopic
        orig_uri = os.environ['ROS_MASTER_URI']
        os.environ['ROS_MASTER_URI'] = 'http://fake_host:12356'

        try:
            c = 'rostopic'

            try:
                rostopic._rostopic_cmd_list([c, 'list'])
                self.fail("should have raised ROSTopicIOException")
            except rostopic.ROSTopicIOException: pass
            
            try:
                rostopic._rostopic_cmd_info([c, 'info', '/chatter'])
                self.fail("should have raised ROSTopicIOException")
            except rostopic.ROSTopicIOException: pass

            try:
                rostopic._rostopic_cmd_type([c, 'type', '/chatter'])
                self.fail("should have raised ROSTopicIOException")
            except rostopic.ROSTopicIOException: pass

            try:
                rostopic._rostopic_cmd_find([c, 'find', 'std_msgs/String'])
                self.fail("should have raised ROSTopicIOException")
            except rostopic.ROSTopicIOException: pass

        finally:
            os.environ['ROS_MASTER_URI'] = orig_uri
        
    def test_cmd_type(self):
        import rostopic
        cmd = 'rostopic'
        s = '/rosout_agg'
        t = 'rosgraph_msgs/Log'

        try:
            rostopic.rostopicmain([cmd, 'type', 'fake'])
            self.fail("should have exited")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
                
        for s in ['/chatter', 'chatter', 'foo/chatter', '/bar/chatter']:
            with fakestdout() as b:
                rostopic.rostopicmain([cmd, 'type', s])
                v = b.getvalue().strip()
                self.assertEquals('std_msgs/String', v)

    def test_main(self):
        import rostopic
        c = 'rostopic'
        try:
            rostopic.rostopicmain([c])
            self.fail("should have exited with error")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
        try:
            rostopic.rostopicmain([c, 'foo'])
            self.fail("should have exited with error")
        except SystemExit, e:
            self.assertNotEquals(0, e.code)
        
    def test_cmd_pub(self):
        import rostopic
        cmd = 'rostopic'

        # we can't actually test functional behavior because rostopic
        # needs control over a node, though we can probably change the
        # source code to just check for the existing node.

        invalid = [['-r', '--once', '/foo', 'std_msgs/String', 'hello'],
                   ['-r', 'a', '/foo', 'std_msgs/String', 'hello'],
                   ['-r', '--', '-1', '/foo', 'std_msgs/String', 'hello'],                   
                   [],
                   ['/foo'],
                   ['/foo', 'std_msgs/String', 'a: b: !!c: d: e'],                                      
                   ]
        for i in invalid:
            try:
                rostopic.rostopicmain([cmd, 'pub'] + i)
                self.fail("should have exited with error"+str(i))
            except SystemExit, e:
                self.assert_(e.code != 0)
        

                
    def test_full_usage(self):
        import rostopic
        try:
            rostopic._fullusage()
            self.fail("should have caused system exit")
        except SystemExit: pass

    def test_get_topic_type(self):
        import rostopic
        
        self.assertEquals((None, None, None), rostopic.get_topic_type('/fake', blocking=False))
        
        t, n, f = rostopic.get_topic_type('/rosout', blocking=False)
        self.assertEquals('rosgraph_msgs/Log', t)
        self.assertEquals('/rosout', n)
        self.assert_(f is None)

        t, n, f = rostopic.get_topic_type('/rosout/name', blocking=False)
        self.assertEquals('rosgraph_msgs/Log', t)
        self.assertEquals('/rosout', n)
        self.failIf(f is None)
        from rosgraph_msgs.msg import Log
        self.assertEquals("bob", f(Log(name="bob")))
        
    def test_get_topic_class(self):
        import rostopic
        
        self.assertEquals((None, None, None), rostopic.get_topic_class('/fake'))

        from rosgraph_msgs.msg import Log
        c, n, f = rostopic.get_topic_class('/rosout')
        self.assertEquals(Log, c)
        self.assertEquals('/rosout', n)
        self.assert_(f is None)

        c, n, f = rostopic.get_topic_class('/rosout/name')
        self.assertEquals(c, Log)
        self.assertEquals('/rosout', n)
        self.failIf(f is None)
        self.assertEquals("bob", f(Log(name="bob")))
        
    def test_cmd_info(self):
        import rostopic
        cmd = 'rostopic'

        try:
            rostopic.rostopicmain([cmd, 'info'])
            self.fail("should have exited with error")
        except SystemExit: pass
        try:
            rostopic.rostopicmain([cmd, 'info', '/fake_topic'])
            self.fail("should have exited with error")
        except SystemExit: pass
        try:
            rostopic.rostopicmain([cmd, 'info', '/chatter', '/bar/chatter'])
            self.fail("should have exited with error")
        except SystemExit: pass
        
        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'info', 'rosout'])
            v = b.getvalue()
            for s in ["Publishers:", "Subscribers", "Type: rosgraph_msgs/Log", " * /rosout"]:
                self.assert_(s in v, "failed on %s: %s"%(s, v))
        with fakestdout() as b:            
            rostopic.rostopicmain([cmd, 'info', '/chatter'])
            v = b.getvalue()
            for s in ["Publishers:", "Subscribers", "Type: std_msgs/String", " * /talker"]:
                self.assert_(s in v, "failed on %s: %s"%(s, v))

    def test_cmd_find(self):
        import rostopic
        cmd = 'rostopic'

        try:
            rostopic.rostopicmain([cmd, 'find'])
            self.fail("arg parsing should have failed")
        except SystemExit: pass
        try:
            rostopic.rostopicmain([cmd, 'find', 'std_msgs/String', 'std_msgs/Int32'])
            self.fail("arg parsing should have failed")
        except SystemExit: pass


        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'find', 'std_msgs/String'])
            d = [x for x in b.getvalue().split('\n') if x.strip()]
            v = ['/foo/chatter', '/bar/chatter', '/chatter']
            self.assertEquals(set(v), set(d))

    def callback(self, msg, val):
        self.vals.add(val)
        
    def test_cmd_list(self):
        import rostopic
        cmd = 'rostopic'
        s = '/add_two_ints'

        # test failures
        for invalid in [['-ps'], ['-b' 'file.bag', '-s'], ['-b' 'file.bag', '-p']]:
            try:
                rostopic.rostopicmain([cmd, 'list'] + invalid)
                self.fail("should have failed")
            except SystemExit: pass
        
        # test main entry
        rostopic.rostopicmain([cmd, 'list'])

        # test directly
        topics = ['/chatter', '/foo/chatter', '/bar/chatter', '/rosout', '/rosout_agg'] 

        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'list'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            self.failIf(set(topics)-set(v))

        # publishers-only
        topics = ['/chatter', '/foo/chatter', '/bar/chatter', '/rosout', '/rosout_agg'] 
        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'list', '-p'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            self.failIf(set(topics)-set(v))
            self.failIf('/clock' in v)
            
        # subscribers-only
        topics = ['/rosout'] 
        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'list', '-s'])
            v = [x.strip() for x in b.getvalue().split('\n') if x.strip()]
            self.failIf(set(topics)-set(v), "%s vs. %s"%(topics, v))
            self.failIf('/chatter' in v)

        # turn on verbosity, not checking output as it's not as stable
        with fakestdout() as b:            
            rostopic.rostopicmain([cmd, 'list', '-v'])
            v = b.getvalue()
            self.assert_("Published topics:" in v)
            self.assert_("Subscribed topics:" in v)
            
        with fakestdout() as b:            
            rostopic.rostopicmain([cmd, 'list', '-vs'])
            v = b.getvalue()
            self.failIf("Published topics:" in v)        
            self.assert_("Subscribed topics:" in v)

        with fakestdout() as b:            
            rostopic.rostopicmain([cmd, 'list', '-vp'])
            v = b.getvalue()
            self.assert_("Published topics:" in v)        
            self.failIf("Subscribed topics:" in v)
            
        # test with multiple topic names
        try:
            rostopic.rostopicmain([cmd, 'list', 'rosout', 'rosout_agg'])
            self.fail("should have caused parser error")
        except SystemExit:
            pass

        # test with resolved names
        for n in topics:
            with fakestdout() as b:
                rostopic.rostopicmain([cmd, 'list', n])
                self.assertEquals(n, b.getvalue().strip())

        # test with relative names
        with fakestdout() as b:
            rostopic.rostopicmain([cmd, 'list', 'rosout'])
            self.assertEquals('/rosout', b.getvalue().strip())
            
        # test with namespaces
        with fakestdout() as b:        
            rostopic.rostopicmain([cmd, 'list', '/foo'])
            self.assertEquals('/foo/chatter', b.getvalue().strip())
        with fakestdout() as b:        
            rostopic.rostopicmain([cmd, 'list', 'bar'])
            self.assertEquals('/bar/chatter', b.getvalue().strip())

NAME = 'test_rostopic'
if __name__ == '__main__':
    rostest.unitrun('test_rostopic', NAME, TestRostopic, sys.argv, coverage_packages=['rostopic'])
