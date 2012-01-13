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
import traceback

import yaml

import roslib.message
import genpy
from genpy import Time, Duration

# Not much to test, just tripwires

class MessageTest(unittest.TestCase):
    
    def test_check_types_Header(self):
        # #2128: test that check_types works with a Header
        # #message. This is a weird case because Header has an aliased
        # #type and is the only ROS type for which that is true
        from test_roslib_comm.msg import HeaderTest
        x = HeaderTest()
        x._check_types()
        
    def test_strify_message(self):
        from genpy.message import Message, strify_message, fill_message_args
        def roundtrip(m):
            yaml_text = strify_message(m)
            print yaml_text
            loaded = yaml.load(yaml_text) 
            print "loaded", loaded
            new_inst = m.__class__()
            if loaded is not None:
                fill_message_args(new_inst, [loaded])
            else:
                fill_message_args(new_inst, [])                
            return new_inst

        # The following tests have not been ported to genpy yet

        # test array of Messages field. We can't use M4 or M5 because fill_message_args has to instantiate the embedded type
        from test_roslib_comm.msg import ArrayOfMsgs
        from std_msgs.msg import String, Time, MultiArrayLayout, MultiArrayDimension
        dims1 = [MultiArrayDimension(*args) for args in [('', 0, 0), ('x', 1, 2), ('y of z', 3, 4)]]
        dims2 = [MultiArrayDimension('hello world', 91280, 1983274)]
        times = [Time(genpy.Time(*args)) for args in [(0,), (12345, 6789), (1, 1)]]
        val = ArrayOfMsgs([String(''), String('foo'), String('bar of soap')],
                          times,
                          [MultiArrayLayout(dims1, 0), MultiArrayLayout(dims2, 12354)],
                          )
        self.assertEquals(val, roundtrip(val))
        

    def test_get_message_class(self):
        from roslib.message import get_message_class
      
        try:
            self.assertEquals(None, get_message_class('String'))
            self.fail("should have thrown ValueError")
        except ValueError: pass
        # non-existent package
        self.assertEquals(None, get_message_class('fake/Fake'))
        # non-existent message
        self.assertEquals(None, get_message_class('roslib/Fake'))
        # package with no messages
        self.assertEquals(None, get_message_class('genmsg_cpp/Fake'))
    
        import rosgraph_msgs.msg
        import std_msgs.msg
        self.assertEquals(std_msgs.msg.Header, get_message_class('Header'))
        self.assertEquals(std_msgs.msg.Header, get_message_class('std_msgs/Header'))
        self.assertEquals(rosgraph_msgs.msg.Log, get_message_class('rosgraph_msgs/Log'))    

    def test_get_service_class(self):
        from roslib.message import get_service_class

        # non-existent package
        self.assertEquals(None, get_service_class('fake/Fake'))
        # non-existent message
        self.assertEquals(None, get_service_class('roslib/Fake'))
        # package with no messages
        self.assertEquals(None, get_service_class('genmsg_cpp/Fake'))
    
        import std_srvs.srv 
        self.assertEquals(std_srvs.srv.Empty, get_service_class('std_srvs/Empty'))    

