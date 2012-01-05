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

PKG = 'test_rospy'
NAME = 'test_latch'

import sys
import time
import unittest

from std_msgs.msg import String

class TestLatch(unittest.TestCase):
        
    def setUp(self):
        self.callback_invoked = {}
        for i in range(0, 6):
            self.callback_invoked[i] = False
        
    def callback_args(self, msg, i):
        self.assertEquals('foo', msg.data)
        self.callback_invoked[i] = True
        
    def callback(self, msg):
        self.assertEquals('foo', msg.data)
        self.callback_invoked[0] = True

    def test_latch(self):
        import rospy
        
        # multi-part test. First check that we get latched message, then check
        # that subscribers to same topic also receive latched message
        # #1852
        rospy.init_node(NAME)
        s0 = rospy.Subscriber('s', String, self.callback)
        # 20 seconds to receive first latched message
        timeout_t = time.time() + 20.
        print "waiting for 20 seconds"
        while not self.callback_invoked[0] and \
                not rospy.is_shutdown() and \
                timeout_t > time.time():
            time.sleep(0.2)

        self.failIf(timeout_t < time.time(), "timeout exceeded")
        self.failIf(rospy.is_shutdown(), "node shutdown")            
        self.assert_(self.callback_invoked[0], "callback not invoked")
        
        # register three more callbacks, make sure they get invoked with message
        # - callbacks are actually called inline, but in spirit of test, async callback is allowed
        for i in range(1, 5):
            self.failIf(self.callback_invoked[i])
            s = rospy.Subscriber('s', String, self.callback_args, i)
            timeout_t = time.time() + 0.5
            while not self.callback_invoked[i] and \
                    not rospy.is_shutdown() and \
                    timeout_t > time.time():
                time.sleep(0.1)
            self.assert_(self.callback_invoked[i])
        
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestLatch, sys.argv)
