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

## Integration test for empty services to test serializers
## and transport

PKG = 'test_rospy'
NAME = 'test_embed_msg'

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String, Int32
from test_rospy.msg import EmbedTest, Val, ArrayVal

PUBTOPIC = "chatter"
LPNODE = 'listenerpublisher'
LPTOPIC = 'listenerpublisher'

MSG = EmbedTest

TIMEOUT = 10.0 #seconds

class TestEmbedMsg(unittest.TestCase):

    def setUp(self):
        self.callback_data = None
        
    def _test_embed_msg_callback(self, data):
        self.callback_data = data
    
    def test_embed_msg(self):
        self.assert_(self.callback_data is None, "invalid test fixture")

        # wait at most 5 seconds for listenerpublisher to be registered
        timeout_t = time.time() + 5.0
        while not rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)) and time.time() < timeout_t:
            time.sleep(0.1)

        self.assert_(rostest.is_subscriber(
            rospy.resolve_name(PUBTOPIC),
            rospy.resolve_name(LPNODE)), "%s is not up"%LPNODE)
        
        print "Publishing to ", PUBTOPIC
        pub = rospy.Publisher(PUBTOPIC, MSG)
        rospy.Subscriber(LPTOPIC, MSG, self._test_embed_msg_callback) 

        # publish about 10 messages for fun
        import random
        val = random.randint(0, 109812312)
        msg = "hi [%s]"%val
        for i in xrange(0, 10):
            # The test message could be better in terms of the values
            # it assigns to leaf fields, but the main focus is trying
            # to dig up edge conditions in the embeds, especially with
            # respect to arrays and embeds.
            pub.publish(
                MSG(String(msg), Int32(val),
                    [Int32(val+1), Int32(val+2), Int32(val+3)],
                    Val(msg+msg),
                    [Val(msg), Val("two")],
                    [ArrayVal([Val("av1"), Val("av2")]), #[Val("%s"%i) for i in xrange(0, 10)]),
                     ArrayVal([]) #,[Val("%s"%i) for i in xrange(0, 10)]),
                     ]
                    ))
            time.sleep(0.1)

        # listenerpublisher is supposed to repeat our messages back onto /listenerpublisher,
        # make sure we got it
        self.assert_(self.callback_data is not None, "no callback data from listenerpublisher")
        print "Got ", self.callback_data.str1.data, self.callback_data.int1.data
        errorstr = "callback msg field [%s] from listenerpublisher does not match"
        self.assertEquals(msg, self.callback_data.str1.data,
                          errorstr%"str1.data")
        self.assertEquals(val, self.callback_data.int1.data,
                          errorstr%"int1.data")
        for i in xrange(1, 4):
            self.assertEquals(val+i, self.callback_data.ints[i-1].data,
                              errorstr%"ints[i-1].data")
        self.assertEquals(msg+msg, self.callback_data.val.val,
                          errorstr%"val.val")
        self.assertEquals(msg, self.callback_data.vals[0].val,
                          errorstr%"vals[0].val")
        self.assertEquals("two", self.callback_data.vals[1].val,
                          errorstr%"vals[1].val")
        # #435: test array of arrays
        self.assertEquals(2, len(self.callback_data.arrayval),
                          errorstr%"len arrayval")
        self.assertEquals(2, len(self.callback_data.arrayval[0].vals),
                          errorstr%"len arrayval[0].vals")
        self.assertEquals("av1", self.callback_data.arrayval[0].vals[0].val,
                          errorstr%"arrayval[0].vals[0].val")
        self.assertEquals("av2", self.callback_data.arrayval[0].vals[1].val,
                          errorstr%"arrayval[0].vals[1].val")
        self.assertEquals(0, len(self.callback_data.arrayval[1].vals),
                          errorstr%"len arrayval[1].vals")

        
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestEmbedMsg, sys.argv)
