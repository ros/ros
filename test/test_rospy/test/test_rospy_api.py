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

import os
import sys
import struct
import unittest
import time

# test rospy API verifies that the rospy module exports the required symbols
class TestRospyApi(unittest.TestCase):

    def test_msg(self):
        # rospy.Message really only exists at the client level, as the internal
        # implementation is built around the roslib reference, so we put the test here

        import rospy
        #trip wires against Message API
        m = rospy.Message()
        from cStringIO import StringIO
        buff = StringIO()
        m.serialize(buff)
        self.assertEquals(0, buff.tell())
        m.deserialize('')
        
    def test_anymsg(self):
        # rospy.AnyMsg really only exists at the client level as nothing within
        # rospy uses its functionality.
        

        from cStringIO import StringIO
        import rospy
        #trip wires against AnyMsg API
        m = rospy.AnyMsg()
        try:
            m.serialize(StringIO())
            self.fail("AnyMsg should not allow serialization")
        except:
            pass

        teststr = 'foostr-%s'%time.time()
        m.deserialize(teststr)
        self.assertEquals(teststr, m._buff)

        #test AnyMsg ctor error checking
        try:
            m = rospy.AnyMsg('foo')
            self.fail("AnyMsg ctor should not allow args")
        except: pass

    def test_rospy_api(self):
        import rospy

        # just a laundry list of API methods to make sure that they still exist
        
        # removed
        try:
            rospy.add_shutdown_hook
            self.fail("add_shutdown_hookshould not longer be top-level API")
        except AttributeError: pass

        rospy.DEBUG
        rospy.INFO
        rospy.WARN
        rospy.ERROR
        rospy.FATAL
        
        rospy.get_caller_id
        rospy.get_name        
        rospy.get_master
        rospy.get_namespace
        rospy.get_published_topics
        rospy.get_node_uri
        rospy.get_ros_root
        rospy.get_time
        rospy.get_rostime
        rospy.init_node
        rospy.is_shutdown
        rospy.logdebug
        rospy.logerr
        rospy.logfatal
        rospy.loginfo        
        rospy.logout #deprecated
        rospy.logwarn
        rospy.myargv
        rospy.on_shutdown
        rospy.parse_rosrpc_uri
        rospy.resolve_name
        rospy.remap_name
        rospy.signal_shutdown
        rospy.sleep
        rospy.spin
        rospy.wait_for_message
        rospy.wait_for_service

        rospy.delete_param
        rospy.get_param
        rospy.get_param_names
        rospy.has_param
        rospy.set_param
        rospy.search_param        

        rospy.AnyMsg
        rospy.Duration
        rospy.Header
        rospy.MasterProxy
        rospy.Message
        rospy.Publisher
        rospy.Rate
        rospy.ROSException
        rospy.ROSInternalException
        rospy.ROSSerializationException
        rospy.ServiceException
        rospy.Service
        rospy.ServiceProxy
        rospy.SubscribeListener
        rospy.Subscriber        
        rospy.Time
        rospy.TransportException
        rospy.TransportTerminated
        rospy.TransportInitError
