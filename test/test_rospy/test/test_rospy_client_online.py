#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id: test_rospy_core.py 4218 2009-04-16 21:09:31Z sfkwc $

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time
import random

import rostest

from threading import Thread
class TestTask(Thread):
    def __init__(self, task):
        Thread.__init__(self)
        self.task = task
        self.success = False
        self.done = False
        self.value = None
        
    def run(self):
        try:
            print "STARTING TASK"
            self.value = self.task()
            print "TASK HAS COMPLETED"
            self.success = True
        except:
            import traceback
            traceback.print_exc()
        self.done = True

class TestRospyClientOnline(unittest.TestCase):
    
    def __init__(self, *args):
        unittest.TestCase.__init__(self, *args)
        import rospy
        rospy.init_node('test_rospy_online')

    def test_wait_for_service(self):
        # lazy-import for coverage
        import rospy
        import time

        # test wait for service in success case        
        def task1():
            rospy.wait_for_service('add_two_ints')
        timeout_t = time.time() + 5.
        t1 = TestTask(task1)
        t1.start()
        while not t1.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t1.success)
        
        # test wait for service with timeout in success case
        def task2():
            rospy.wait_for_service('add_two_ints', timeout=1.)
        timeout_t = time.time() + 5.        
        t2 = TestTask(task2)        
        t2.start()
        while not t2.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t2.success)

        # test wait for service in failure case
        def task3():
            # #2842 raising bounds from .1 to .3 for amazon VM            
            rospy.wait_for_service('fake_service', timeout=0.3)
        timeout_t = time.time() + 2.        
        t3 = TestTask(task3)        
        t3.start()
        while not t3.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t3.done)
        self.failIf(t3.success)
    
    def test_ServiceProxy_wait_for_service(self):
        """
        Test ServiceProxy.wait_for_service
        """
        # lazy-import for coverage
        import rospy
        import time
        import test_ros.srv

        # test wait for service in success case        
        proxy = rospy.ServiceProxy('add_two_ints', test_ros.srv.AddTwoInts)
        class ProxyTask(object):
            def __init__(self, proxy, timeout=None):
                self.proxy = proxy
                self.timeout = timeout
            def __call__(self):
                if self.timeout is None:
                    self.proxy.wait_for_service()
                else:
                    self.proxy.wait_for_service(timeout=self.timeout)
        timeout_t = time.time() + 5.
        t1 = TestTask(ProxyTask(proxy))
        t1.start()
        while not t1.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t1.success)
        
        # test wait for service with timeout in success case
        timeout_t = time.time() + 5.        
        t2 = TestTask(ProxyTask(proxy, timeout=1.))
        t2.start()
        while not t2.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t2.success)

        # test wait for service in failure case
        fake_proxy = rospy.ServiceProxy('fake_service', test_ros.srv.AddTwoInts)
        timeout_t = time.time() + 2.        
        t3 = TestTask(ProxyTask(fake_proxy, timeout=0.1))        
        t3.start()
        while not t3.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t3.done)
        self.failIf(t3.success)

    def test_sleep(self):
        import rospy
        import time
        t = time.time()
        rospy.sleep(0.1)
        dur = time.time() - t
        # #2842 raising bounds from .01 to .03 for amazon VM 

        # make sure sleep is approximately right
        self.assert_(abs(dur - 0.1) < 0.03, dur)

        t = time.time()
        rospy.sleep(rospy.Duration.from_sec(0.1))
        dur = time.time() - t
        # make sure sleep is approximately right
        self.assert_(abs(dur - 0.1) < 0.03, dur)

        # sleep for neg duration
        t = time.time()
        rospy.sleep(rospy.Duration.from_sec(-10.))
        dur = time.time() - t
        # make sure returned immediately
        self.assert_(abs(dur) < 0.1, dur)

    def test_Rate(self):
        import rospy
        import time
        t = time.time()
        count = 0
        r = rospy.Rate(10)
        for x in xrange(10):
            r.sleep()
        dur = time.time() - t
        # make sure sleep is approximately right
        self.assert_(abs(dur - 1.0) < 0.5, dur)
        

    def test_param_server(self):
        # this isn't a parameter server test, just checking that the rospy bindings work
        import rospy

        try:
            rospy.get_param('not_a_param')
            self.fail("should have raised KeyError")
        except KeyError: pass
        self.assertEquals('default_val', rospy.get_param('not_a_param', 'default_val') )
        
        p = rospy.get_param('/param')
        self.assertEquals("value", p)
        p = rospy.get_param('param')
        self.assertEquals("value", p)
        p = rospy.get_param('/group/param')
        self.assertEquals("group_value", p)
        p = rospy.get_param('group/param')
        self.assertEquals("group_value", p)

        self.assertEquals('/param', rospy.search_param('param'))
        
        names = rospy.get_param_names()
        self.assert_('/param' in names)
        self.assert_('/group/param' in names)        

        for p in ['/param', 'param', 'group/param', '/group/param']:
            self.assert_(rospy.has_param(p))
            
        rospy.set_param('param2', 'value2')
        self.assert_(rospy.has_param('param2'))
        self.assertEquals('value2', rospy.get_param('param2'))
        rospy.delete_param('param2')
        self.failIf(rospy.has_param('param2'))
        try:
            rospy.get_param('param2')
            self.fail("should have raised KeyError")
        except KeyError: pass

    def test_wait_for_message(self):
        # lazy-import for coverage
        import rospy
        import std_msgs.msg
        import time

        # test standard wait for message
        def task1():
            return rospy.wait_for_message('chatter', std_msgs.msg.String)
        timeout_t = time.time() + 5.
        t1 = TestTask(task1)
        t1.start()
        while not t1.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t1.success)
        self.assert_('hello' in t1.value.data)

        # test wait for message with timeout
        def task2():
            return rospy.wait_for_message('chatter', std_msgs.msg.String, timeout=2.)
        timeout_t = time.time() + 5.        
        t2 = TestTask(task2)        
        t2.start()
        while not t2.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.assert_(t2.success)
        self.assert_('hello' in t2.value.data)
        
        # test wait for message with timeout FAILURE
        def task3():
            # #2842 raising bounds from .1 to .3 for amazon VM
            return rospy.wait_for_message('fake_topic', std_msgs.msg.String, timeout=.3)
        timeout_t = time.time() + 2.        
        t3 = TestTask(task3)        
        t3.start()
        while not t3.done and time.time() < timeout_t:
            time.sleep(0.5)
        self.failIf(t3.success)
        self.assert_(t3.done)
        self.assert_(t3.value is None)
    
if __name__ == '__main__':
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyClientOnline, coverage_packages=['rospy.client', 'rospy.msproxy'])
