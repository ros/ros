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
NAME = 'test_pubsub_order'

import sys 
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String

PUBTOPIC = "chatter"
LPNODE = 'listenerpublisher'
LPTOPIC = 'listenerpublisher'
MSG = String

TIMEOUT = 10.0 #seconds

class TestPubSubOrder(unittest.TestCase):

    def setUp(self):
        self.callback_data = None
        
    def _test_subscriber_first_callback(self, data):
        self.callback_data = data
    
    ## Test subscriber first makes sure that if a subscriber is up first
    ## that it is able to successfully receive messages from a new publisher
    def test_subscriber_first(self):
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
        rospy.Subscriber(LPTOPIC, MSG, self._test_subscriber_first_callback) 

        # publish about 10 messages for fun
        import random
        val = random.randint(0, 109812312)
        msg = "hi [%s]"%val
        for i in xrange(0, 10):
            pub.publish(MSG(msg))
            time.sleep(0.1)

        # listenerpublisher is supposed to repeat our messages back onto /listenerpublisher,
        # make sure we got it
        self.assert_(self.callback_data is not None, "no callback data from listenerpublisher")
        self.assertEquals(msg, self.callback_data.data, "callback data from listenerpublisher does not match")
        
if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.run(PKG, NAME, TestPubSubOrder, sys.argv)
