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
# Revision $Id$

import roslib; roslib.load_manifest('test_rospy')

import os
import sys
import struct
import unittest
import time
import random

import rostest

import rospy


class TestRospyCore(unittest.TestCase):
    
    def test_parse_rosrpc_uri(self):
        from rospy.core import parse_rosrpc_uri
        valid = [('rosrpc://localhost:1234/', 'localhost', 1234), 
                 ('rosrpc://localhost2:1234', 'localhost2', 1234), 
                 ('rosrpc://third:1234/path/bar', 'third', 1234), 
                 ('rosrpc://foo.com:1/', 'foo.com', 1),
                 ('rosrpc://foo.com:1/', 'foo.com', 1)]
        for t, addr, port in valid:
            paddr, pport = rospy.core.parse_rosrpc_uri(t)
            self.assertEquals(addr, paddr)
            self.assertEquals(port, pport)
            # validate that it's a top-level API method
            self.assertEquals(rospy.core.parse_rosrpc_uri(t), rospy.parse_rosrpc_uri(t))
        invalid = ['rosrpc://:1234/', 'rosrpc://localhost', 'http://localhost:1234/']
        for i in invalid:
            try:
                parse_rosrpc_uri(i)
                self.fail("%s was an invalid rosrpc uri"%i)
            except: pass
        
    def test_add_log_handler(self):
        # tripwire test
        import rospy.core
        import roslib.msg
        for level in [roslib.msg.Log.DEBUG, 
                      roslib.msg.Log.INFO,
                      roslib.msg.Log.WARN,
                      roslib.msg.Log.ERROR,
                      roslib.msg.Log.FATAL]:
            rospy.core.add_log_handler(level, lambda x: x)
        import rospy.exceptions
        try:
            rospy.core.add_log_handler(lambda x: x, -1)
        except rospy.exceptions.ROSInternalException, e: pass

    def test_loggers(self):
        # trip wire tests
        import rospy.core
        rospy.core.logdebug('debug')
        rospy.core.logwarn('warn')
        rospy.core.logout('out')
        rospy.core.logerr('err')
        rospy.core.logfatal('fatal')
        # test that they are exposed via top-level api
        import rospy
        rospy.logdebug('debug')
        rospy.logwarn('warn')
        rospy.logout('out')
        rospy.logerr('err')
        rospy.logfatal('fatal')
        
    def test_add_shutdown_hook(self):
        def handle(reason):
            pass
        # cannot verify functionality, just crashing
        rospy.core.add_shutdown_hook(handle)
        try:
            rospy.core.add_shutdown_hook(1)
            self.fail_("add_shutdown_hook is not protected against invalid args")
        except TypeError: pass
        try:
            rospy.core.add_shutdown_hook(1)
            self.fail_("add_shutdown_hook is not protected against invalid args")
        except TypeError: pass

    def test_add_preshutdown_hook(self):
        def handle1(reason):
            pass
        def handle2(reason):
            pass
        def handle3(reason):
            pass
        # cannot verify functionality, just coverage as well as ordering
        rospy.core.add_shutdown_hook(handle1)
        rospy.core.add_shutdown_hook(handle2)
        rospy.core.add_preshutdown_hook(handle3)
        self.assert_(handle3 in rospy.core._preshutdown_hooks)
        self.assert_(handle2 in rospy.core._shutdown_hooks)        
        self.assert_(handle1 in rospy.core._shutdown_hooks)        
        try:
            rospy.core.add_preshutdown_hook(1)
            self.fail_("add_preshutdown_hook is not protected against invalid args")
        except TypeError: pass
        try:
            rospy.core.add_preshutdown_hook(1)
            self.fail_("add_preshutdown_hook is not protected against invalid args")
        except TypeError: pass

    def test_get_ros_root(self):
        try:
            rospy.core.get_ros_root(env={}, required=True)
        except:
            pass
        self.assertEquals(None, rospy.core.get_ros_root(env={}, required=False))
        rr = "%s"%time.time()
        self.assertEquals(rr, rospy.core.get_ros_root(env={'ROS_ROOT': rr}, required=False))
        self.assertEquals(rr, rospy.core.get_ros_root(env={'ROS_ROOT': rr}, required=True))        

        self.assertEquals(os.environ['ROS_ROOT'], rospy.core.get_ros_root(required=False))
    def test_node_uri(self):
        uri = "http://localhost-%s:1234"%random.randint(1, 1000)
        self.assertEquals(None, rospy.core.get_node_uri())
        rospy.core.set_node_uri(uri)
        self.assertEquals(uri, rospy.core.get_node_uri())
        
    def test_initialized(self):
        self.failIf(rospy.core.is_initialized())
        rospy.core.set_initialized(True)
        self.assert_(rospy.core.is_initialized())

    def test_shutdown_hook_exception(self):
        rospy.core._shutdown_flag = False
        del rospy.core._shutdown_hooks[:]
        # add a shutdown hook that throws an exception,
        # signal_shutdown should be robust to it
        rospy.core.add_shutdown_hook(test_shutdown_hook_exception)
        rospy.core.signal_shutdown('test_exception')
        rospy.core._shutdown_flag = False        
        del rospy.core._shutdown_hooks[:]
        
    def test_shutdown(self):
        rospy.core._shutdown_flag = False
        del rospy.core._shutdown_hooks[:]        
        global called, called2
        called = called2 = None
        self.failIf(rospy.core.is_shutdown())        
        rospy.core.add_shutdown_hook(test_shutdown_hook)
        reason = "reason %s"%time.time()
        rospy.core.signal_shutdown(reason)
        self.assertEquals(reason, called)
        self.assert_(rospy.core.is_shutdown())

        # verify that shutdown hook is called immediately on add if already shutdown
        rospy.core.add_shutdown_hook(test_shutdown_hook2)
        self.assert_(called2 is not None)
        rospy.core._shutdown_flag = False

    #TODO: move to teset_rospy_names
    def test_valid_name(self):
        # not forcing rospy to be pedantic -- yet, just try and do sanity checks
        tests = ['/', 'srv', '/service', '/service1', 'serv/subserv']
        caller_id = '/me'
        for t in tests:
            self.assert_(rospy.core.valid_name('p')(t, caller_id))
        failures = ['ftp://foo', '', None, 1, True, 'http:', ' spaced ', ' ']
        for f in failures:
            try:
                rospy.core.valid_name('p')(f, caller_id)
                self.fail(f)
            except rospy.core.ParameterInvalid, e:
                pass
    
    def test_is_topic(self):
        # not forcing rospy to be pedantic -- yet, just try and do sanity checks
        caller_id = '/me'
        tests = [
            ('topic', '/node', '/topic'),
            ('topic', '/ns/node', '/ns/topic'),
            ('/topic', '/node', '/topic'),
            ('~topic', '/node', '/node/topic'),
            ('/topic1', '/node', '/topic1'),
            ('top/sub', '/node', '/top/sub'),
            ('top/sub', '/ns/node', '/ns/top/sub'),
            ]

        for t, caller_id, v in tests:
            self.assertEquals(v, rospy.core.is_topic('p')(t, caller_id))
        failures = ['/', 'ftp://foo', '', None, 1, True, 'http:', ' spaced ', ' ']
        for f in failures:
            try:
                rospy.core.is_topic('p')(f, caller_id)
                self.fail(f)
            except rospy.core.ParameterInvalid, e:
                pass
            
    def test_configure_logging(self):
        # can't test actual functionality
        try:
            rospy.core.configure_logging("/")
            self.fail("configure_logging should not accept a the root namespace as the node_name param")
        except: pass
        rospy.core.configure_logging("/node/name")

    def test_xmlrpcapi(self):
        # have to use 'is' so we don't accidentally invoke XMLRPC
        self.assert_(rospy.core.xmlrpcapi(None) is None)
        self.assert_(rospy.core.xmlrpcapi('localhost:1234') is None)
        self.assert_(rospy.core.xmlrpcapi('http://') is None)
        api = rospy.core.xmlrpcapi('http://localhost:1234')
        api2 = rospy.core.xmlrpcapi('http://localhost:1234')
        # verify cache
        self.assert_(api is api2)
    
called = None
called2 = None
def test_shutdown_hook(reason):
    global called
    print "HOOK", reason
    called = reason
def test_shutdown_hook2(reason):
    global called2
    print "HOOK2", reason
    called2 = reason
def test_shutdown_hook_exception(reason):
    raise Exception("gotcha")

if __name__ == '__main__':
    rostest.unitrun('test_rospy', sys.argv[0], TestRospyCore, coverage_packages=['rospy.core'])
